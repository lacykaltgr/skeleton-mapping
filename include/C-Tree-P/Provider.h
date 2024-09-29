// Pontelh� feldolgoz� �s akad�ly felismer� alkalmaz�s - (C) Czimber M�rk 2023
//--------------------------------
#include "Page.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ctime>  // For time measurement

// Pontfelh� bet�lt�, t�rol�, feldolgoz� oszt�ly
class Provider
{
private:
	// bels� v�ltoz�k: 
	Page mRoot; //pontok lapjai els�/utols�
	int8 pNumber; // pontsz�m
	Bound mBound; // 3D befogal�
	pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud; // pcl cloud

public:
	/*
	 * Az oszt�ly param�ter n�lk�li konstruktora inicializ�lja a v�ltoz�kat 0-val.
	 */
	Provider() : mCloud(new pcl::PointCloud<pcl::PointXYZ>), pNumber(0) {}

	/*
	 * Az oszt�ly destruktora a Delete f�ggv�nyt h�vja.
	 * A Delete f�ggv�ny t�rli a pontokat, ezenk�v�l a domborzat befogal�t �s a voxeleket.
	 * Felszabad�t� f�ggv�ny dinamikus mem�ri�hoz.
	 */
	~Provider()
	{
		Erase();
	}

	void Erase()
	{
		mRoot.Erase();
		pNumber = 0;
	}
	
	// pont a poligonban vizsg�lat
	static bool PointInPolygon(int num, Point *poly, Point &point)
	{
		int		i, cross;
		bool	yflag0, yflag1, xflag0;
		double	x, y, nx, ny;

		if(num<3 || poly==0) return 0; // hib�s poligon
		x=poly[num-1].x;  y=poly[num-1].y;  cross=0; // inicializ�l�s
		yflag0=( y >= point.y );  // kezd�pont a +X tengely f�l�tt/alatt

		for(i=0;  i < num;  i++) // t�r�spontok
		{
			nx=poly[i].x;  ny=poly[i].y; // k�vetkez� t�r�spont
			if(nx==point.x && ny==point.y) return 0; // pont �s t�r�spont egybeesik - nincs metsz�s
			yflag1=( ny >= point.y ); // v�gpont a +X tengely f�l�tt/alatt
			if( yflag0 != yflag1 ) // v�gpontok a +X tengely ellent�tes oldal�n vannak
			{
				xflag0=( x >= point.x ); // kezd�pont a +Y tengely jobb/bal oldal�n
				if( xflag0 == ( nx >= point.x ) ) // mindk�t v�gpont azonos oldalon
				{
					if(xflag0) cross++; // mindk�t v�gpont a jobb oldalon
				}
				else // k�l�nben metsz�spont sz�m�t�s, line�ris interpol�ci�
				{
					if( (nx - (ny - point.y) * ( x - nx) / (y - ny)) >= point.x ) cross++;
				}
			}
			yflag0=yflag1;  x=nx;  y=ny; // v�ndorl�s
		} // end for verteces
		return cross & 1; // p pontb�l kiindul� +X tengely h�nyszor metszi a poligon oldalakat, Jordan Curve (Preparata 1985, Glassner 1989)
	}
	
	// pontfelh� tiszt�t�s, pont vizsg�lat, hogy a pont az utca k�z�ps� r�sz�n van-e
	bool IsPointInStreet(Point &pt)
	{
		static Point poly[7]= // utca poligon
		{
			{ 652820.62, 238253.07, 0 },
			{ 652945.74, 238270.80, 0 },
			{ 653022.26, 238280.16, 0 },
			{ 653020.77, 238276.72, 0 },
			{ 652931.35, 238264.73, 0 },
			{ 652930.57, 238261.88, 0 },
			{ 652821.18, 238247.12, 0 }
		};
		
		// pont a poligonban vizsg�lat
		if(PointInPolygon(7, poly, pt)==0) return 0;
		
		// magass�g vizsg�lat k�t szakaszra elt�r� lejt�ssel
		if(pt.x < 652917.4)
		{
			if(pt.z < 0.0137332749*pt.x - 8855.76) return 0; // utca s�kja
		}
		else
		{
			if(pt.z < 0.0069960127*pt.x - 4456.86) return 0; // utca s�kja
		}
		return 1; // a pontot t�r�lni kell
	}

	/*
	 * A ReadLAS f�ggv�ny felel�s a bin�ris pontfelh� �llom�ny beolvas�s��rt.
	 * A pontokat (Point) mem�ri�ban list�ba f�z�tt lapokon (Page) t�rolja.
	 * Param�ter: filename - a bin�ris f�jl neve
	 * Visszat�r�si �rt�k: 0 - sikertelen, 1 - sikeres beolvas�s
	 */
	bool ReadLAS(const char *filename) // pontok bet�lt�se Las filebol
	{
		LasHead head;
		LasPoint point;
		Point pt;
		int recordSize;
		bool readSucces = 1;
		int num = 0;

		// header bet�lt�se Las-bol
		ifstream f; // f�jl input objektum
		f.open(filename, ios::in | ios::binary); // megnyit�s
		if(f.is_open() == 0) return 0; // teszt, hogy siker�lt-e megnyitni
		f.read((char *)&head, sizeof(LasHead)); // bin�ris olvas�s data v�ltoz�ba size m�ret
		if(f.eof()) { f.close(); return 0; } // ellen�rizni, hogy siker�lt e a beolvas�s
		if(head.Signature != 0x4653414C || head.VersionMajor != 1) { f.close();  return 0; }

		recordSize = head.PointDataRecordLength; // pont record m�rete 
		pNumber = head.LegacyNumberOfPointRecords; // pontok sz�ma
		if(pNumber == 0) pNumber = head.NumberOfPointRecords;

		f.seekg(head.OffsetToPointData, ios::beg); // offset to point dat�ra poz�cion�lja a file pointert
		// a file elej�t�l 
		while(readSucces) // am�g az olvas�s sikeres
		{
			f.read((char *)&point, recordSize); // f olvas�sa mint pont, pont hossz�s�g� param�tert v�r
			num++; // seg�d sz�ml�l� 
			if(f.fail() || num == pNumber) // ha hib�s olvas�s, vagy v�ge a pontok sz�m�nak
			{
				readSucces = false;
				pNumber = num;
				break;
			}

			pt.x = point.x * head.XScaleFactor + head.XOffset; // x,y,z koordin�t�k sz�mol�sa *scale + offset
			pt.y = point.y * head.YScaleFactor + head.YOffset;
			pt.z = point.z * head.ZScaleFactor + head.ZOffset;

			if(IsPointInStreet(pt)==1) continue; // tiszt�t�s, nincs hozz�ad�s, utca k�z�ps� r�sz�n van a pont
			// ellen�rz�s minim�lis id�t vesz ig�nybe

			if(mRoot.Add(pt) == false) return false; // pont hozz�ad�sa r-tree indexhez
			mBound.Extend(pt, num-1); // befoglal� sz�m�t�s

		}
		f.close(); return true;
	}

	double ReadPCD(const char* filename) // Function to load points from PCD file
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *mCloud) == -1)
		{
			cerr << "Couldn't read file " << filename << std::endl;
			return false;
		}

		cout << "Loaded " << mCloud->width * mCloud->height << " data points!" << std::endl;

		clock_t start = clock();  // Start measuring time for indexing

		int num = 0; // To keep track of the point number
		for (const auto& point : *mCloud)
		{
			Point pt; // Assuming your Point struct exists with x, y, z members
			pt.x = point.x;
			pt.y = point.y;
			pt.z = point.z;

			// if (IsPointInStreet(pt) == 1) continue; // Filtering points if necessary

			if (mRoot.Add(pt) == false) return false; // Adding the point to the r-tree index
			mBound.Extend(pt, num); // Extending the bounding volume

			num++;
		}

		clock_t end = clock();  // End measuring time for indexing
		double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
		cout << "Indexed " << num << " points in " << cpu_time_used << " seconds." << std::endl;

		return cpu_time_used;
	}

	void GetBound(Bound &bound)
	{
		bound = mBound;
	}

	// Pontok lek�rdez�se befoglal�ra
	bool QueryPoints(Bound &bound, vector<Point> &query)
	{
		query.clear(); 
		return mRoot.Query(bound, query); // �tad�s
	}

	// Pontok lek�rdez�se befoglal�ra szf�rikus m�lys�gi bufferrel (SZB: Spherical Z-Buffer)
	bool QueryPointsSZB(Bound &bound, vector<Point> &query)
	{
		int		i, ix, iy, size=720*360;
		double	x, y, z, cx, cy, cz;
		double	lon, lat, d, s, l;
		Point	*p;
		struct ZPoint { double dist; Point *ref;  ZPoint() { dist=0; ref=0; } };
		ZPoint	*zbuffer, *q;
		vector<Point> inner; // tagv�ltoz� legyen, egyszer foglalni

		zbuffer=new ZPoint[size];  if(zbuffer==0) return 0; // tagv�ltoz� legyen, egyzer foglalni
		query.clear(); // t�mb elemeinek t�rl�se a lefoglalt mem�ria megtart�s�val 

		// egy bels� pontbufferbe k�zeli pontok lek�rdez�se
		if(mRoot.Query(bound, inner)==0) return 0;
		p=inner.data();

		cx=(bound.x1+bound.x2)*0.5;  cy=(bound.y1+bound.y2)*0.5;  cz=(bound.z1+bound.z2)*0.5;

		for(i=0; i<inner.size(); i++,p++)
		{
			// �tt�r�s g�mbi koordin�t�kra
			x=p->x - cx;  y=p->y - cy;  z=p->z - cz; // relat�v
			s=x*x + y*y;  d=sqrt(s + z*z);  l=sqrt(s); // 3D �s 2D hossz
			lon=atan2(y, x)*DEG;  lat=atan2(z, l)*DEG; // g�mbi hossz�s�g �s sz�less�g fokban

			if(lon<-120 || lon>120 || lat<-40 || lat>40) continue; // sz�gtartom�ny sz�k�t�s
			if(d>20) continue; // t�vols�g sz�r�s
			ix=int( (lon+180)*2 );  iy=int( (lat+90)*2 ); // eltol�s, k�z�pen a nulla,nulla pont
			q=zbuffer + iy*720 + ix; // zbuffer cella kiv�laszt�sa
			if(q->ref==0 || d < q->dist) { q->dist=d;  q->ref=p; } // t�rol�s, legk�zelebbi pont
		}

		for(i=0,q=zbuffer; i<size; i++,q++) // lev�logatott pontok
		{
			if(q->ref) query.push_back(*(q->ref));
		}

		delete [] zbuffer;
		//inner.erase();

		return 1;
	}

	Point NearestNeighbourQuery(Point &pt, vector<double> &minDistSq, double maxdist)
	{  
		int i; 
		double d;
		Bound bound;
		vector<Point> points;
		Point nearestPoint;
		minDistSq[0] = 1000000; // Initialize to a large value

		// Expand the search area exponentially until points are found or max distance is reached
		for(d = 1; d <= maxdist; d *= 2)
		{
			// Define the search boundary in a cube around the target point, expanding by `d`
			bound.x1 = pt.x - d;  bound.x2 = pt.x + d;
			bound.y1 = pt.y - d;  bound.y2 = pt.y + d;
			bound.z1 = pt.z - d;  bound.z2 = pt.z + d;

			// Query points within this expanded boundary
			QueryPoints(bound, points);
			if(points.size()) break; // Stop expanding if points are found
		}

		// Now that we have some points, find the nearest one based on Euclidean distance
		for(i = 0; i < points.size(); i++)
		{
			// Euclidean distance squared (no need to take sqrt for comparison)
			double distSq = (pt.x - points[i].x) * (pt.x - points[i].x) +
							(pt.y - points[i].y) * (pt.y - points[i].y) +
							(pt.z - points[i].z) * (pt.z - points[i].z);

			// Update the nearest point if the current point is closer
			if(distSq < minDistSq[0])
			{
				minDistSq[0] = distSq;
				nearestPoint = points[i]; // Store the closest point found
			}
		}

		return nearestPoint; // Return the nearest point found
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloud()
	{
		return mCloud;  // Return the stored point cloud
	}
};