// Pontelh� feldolgoz� �s akad�ly felismer� alkalmaz�s - (C) Czimber M�rk 2023
//--------------------------------
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

#include "Geom.h"
#include "LasFormat.h"

//--------------------------------
using namespace std;

//--------------------------------
// A Page strukt�ra pontok laponk�nti t�rol�s�hoz
class Page
{
    const static int PageSize = 64; // maxim�lis lapm�ret
    //const static int PointSize = 1024; // maxim�lis pontm�ret 1K
    const static int PointSize = 4096; // maxim�lis pontm�ret 8K

    int     mPageNum; // gyereklapok sz�ma
    int     mPointNum; // pontok sz�ma 

    Page *mParent; // sz�l?
    Page *mPages[PageSize]; // gyereklapok
    Point *mPoints; // pontok a legals� lapon
    Bound   mBound; // Lap befoglal�ja

public:
    //----------------
    Page()
    {
        mPageNum = mPointNum = 0;
        mParent = nullptr;
        for(int i = 0; i < PageSize; i++) mPages[i] = nullptr;
        mPoints = nullptr;
    }

    // Todo: Point �s Page generikus hozzz�ad�s megold�sa

    // Todo: Leaf kett�oszt�s k�l�n f�ggv�ny legyen 

    //----------------
    bool Add(Point &point)
    {
        // Ha vannak Pagek, akkor Page kiv�laszt�sa �s pont hozz�ad�sa
        if(mPageNum)
        {
            int     i;
            Bound   s;
            double  terf;
            int     pos; // Page poz�ci�
            double  legk; // legkisebb Page

            mBound.Extend(point, 1); // page befoglal� b?v�t�se

            // point hozz�ad�sa ahhoz a Page-hez ahol a bound t�rfogata a legkevesebbet n�vekszik. 
            for(i = pos = 0; i < mPageNum; i++)
            {
                if(mPages[i]->mBound.Inside(point)) // Pont bel�l van nem vizsg�lunk tov�bb
                    // lehet tov�bb vizsg�lni: kevesebb elemet t�rol�hoz adjuk hozz�
                {
                    pos = i; break;
                }

                s = mPages[i]->mBound;  terf = s.Volume(); // bound t�rfogata
                s.Extend(point, 1);  terf = s.Volume() - terf; // legkisebb t�rfogat n�veked�st keress�k
                if(i == 0) { pos = 0; legk = terf; }
                else if(terf < legk) { pos = i; legk = terf; }
            }
            return mPages[pos]->Add(point); // hozz�adjuk a gyerekhez a pontot
        }

        if(mPoints == 0) // Ha nincs Page �s nincs m�g Points akkor pont fogal�s
        {
            mPoints = new Point[PointSize];
            if(mPoints == 0) return false;
        }

        if(mPointNum < PointSize) // p elhelyez�se Points-ban, ha van hely
        {
            mPoints[mPointNum] = point; // pont t�rol�sa
            mBound.Extend(point, mPointNum++); // befoglal� b?v�t�s, pontsz�m n�vel�s
            return true;
        }
        else // Ha nincs hely a pontnak, akkor page kett�oszt�sa
        {
            int     i;
            char    c;
            double  dx, dy, dz, mx, my, mz, m;
            Page *p, *q;
            Point *s;

            dx = mBound.x2 - mBound.x1;
            dy = mBound.y2 - mBound.y1;
            dz = mBound.z2 - mBound.z1;

            // �tlagpont sz�m�t�sa
            mx = my = mz = 0;
            for(i = 0, s = mPoints; i < PointSize; i++, s++)
            {
                mx += s->x; my += s->y; mz += s->z;
            }
            mx /= PointSize; my /= PointSize; mz /= PointSize;

            if(mParent == 0) // root, k�t �j lap hozz�ad�s root-hoz, root kett�oszt�sa 
            {
                p = new Page;  if(p == nullptr) return 0;
                q = new Page;  if(q == nullptr) return 0;
            }
            else // egy �j lap l�trehoz�sa, hozz�ad�s sz�l?h�z, jelenlegi lap kett�oszt�sa
            {
                p = this;  p->mPointNum = 0;
                q = new Page;  if(q == nullptr) return 0;
            }

            // TODO: kett�oszt�s k�l�n rutin: SplitPoints(Page *p, Page *q)

            if(dx > dy && dx > dz) // x tengely ment�n osztunk
            {
                m = mx; c = 'X';
                for(i = 0, s = mPoints; i < PointSize; i++, s++)
                {
                    if(s->x < mx) p->Add(*s); else q->Add(*s);
                }
            }
            else if(dy > dz) // y tengely ment�n osztunk
            {
                m = my; c = 'Y';
                for(i = 0, s = mPoints; i < PointSize; i++, s++)
                {
                    if(s->y < my) p->Add(*s); else q->Add(*s);
                }
            }
            else // z tengely ment�n osztunk
            {
                m = mz; c = 'Z';
                for(i = 0, s = mPoints; i < PointSize; i++, s++)
                {
                    if(s->z < mz) p->Add(*s); else q->Add(*s);
                }
            }

            //printf("-- DividePoints by %c %.2f: %d/%d\n", c, m, p->mPointNum, q->mPointNum); // kett�oszt�s ki�r�sa DEBUG
            //printf("P: x1=%.2f y1=%.2f x2=%.2f y2=%.2f\n", p->mBound.x1, p->mBound.y1, p->mBound.x2, p->mBound.y2);
            //printf("Q: x1=%.2f y1=%.2f x2=%.2f y2=%.2f\n", q->mBound.x1, q->mBound.y1, q->mBound.x2, q->mBound.y2);

            if(mParent == 0)
            {
                delete[] mPoints;  mPoints = 0;  mPointNum = 0; // pontok t�rl�se
                if(Add(p) == false) return false; // �j lapok hozz�ad�sa
                if(Add(q) == false) return false;
                return Add(point); // hozz�ad�s a root-hoz �jb�l
            }
            else
            {
                if(mParent->Add(q) == false) return false; // �j lap hozz�ad�sa sz�l?h�z
                return mParent->Add(point); // hozz�ad�s sz�l?h�z �jb�l
            }
        }
    }

    //----------------
    bool Add(Page *pg) // �j lap hozz�ad�sa
    {
        int     i;

        if(pg == 0) return 0;
        if(mPageNum < PageSize) // ha van hely, page hozz�ad�sa
        {
            mBound.Extend(pg->mBound, mPageNum); // befoglal� b?v�t�se
            mPages[mPageNum] = pg;  mPageNum++;  pg->mParent = this;
        }
        else // ha nincs, akkor Page kett�oszt�sa
        {
            double  dx, dy, dz, mx, my, mz, m;
            char    c;
            Page *p, *q;
            Point   pt;

            dx = mBound.x2 - mBound.x1;
            dy = mBound.y2 - mBound.y1;
            dz = mBound.z2 - mBound.z1;

            // �tlagpont sz�m�t�sa befoglal�kr
            mx = my = mz = 0;
            for(i = 0; i < PageSize; i++)
            {
                mPages[i]->mBound.GetMean(pt);
                mx += pt.x; my += pt.y; mz += pt.z;
            }
            mx /= PageSize; my /= PageSize; mz /= PageSize;

            if(mParent == 0) // root, k�t �j lap, root kett�oszt�sa 
            {
                p = new Page;  if(p == nullptr) return false;
                q = new Page;  if(q == nullptr) return false;
            }
            else // egy �j lap, jelenlegi lap kett�oszt�sa
            {
                p = this;  p->mPageNum = 0;
                q = new Page;  if(q == nullptr) return false;
            }

            if(dx > dy && dx > dz) // x tengely ment�n osztunk
            {
                c = 'X';  m = mx;
                for(i = 0; i < PageSize; i++)
                {
                    mPages[i]->mBound.GetMean(pt);
                    if(pt.x < mx) p->Add(mPages[i]); else q->Add(mPages[i]);
                }
            }
            else if(dy > dz) // y tengely ment�n osztunk
            {
                c = 'Y';  m = my;
                for(i = 0; i < PageSize; i++)
                {
                    mPages[i]->mBound.GetMean(pt);
                    if(pt.y < my) p->Add(mPages[i]); else q->Add(mPages[i]);
                }
            }
            else // z tengely ment�n osztunk
            {
                c = 'Z';  m = mz;
                for(i = 0; i < PageSize; i++)
                {
                    mPages[i]->mBound.GetMean(pt);
                    if(pt.z < mz) p->Add(mPages[i]); else q->Add(mPages[i]);
                }
            }

            //printf("-- DividePages by %c %.2f: %d/%d\n", c, m, p->mPageNum, q->mPageNum); // kett�oszt�s ki�r�sa DEBUG

            if(mParent == 0) // root, k�t �j lap
            {
                mPageNum = 0; // pages t�mb null�z�sa
                if(Add(p) == false) return false; // p �s q hozz�ad�sa roothoz
                if(Add(q) == false) return false;
            }
            else // egy �j lap hozz�ad�sa sz�l?h�z
            {
                if(mParent->Add(q) == false) return false; // q hozz�ad�sa sz�l?h�z
            }

            // ahhoz a laphoz kell hozz�adni pg-t, amelyn�l kisebb a t�rfogat n�veked�s
            if(Bound::ExtendCost(p->mBound, q->mBound, pg->mBound) == 1) return p->Add(pg);
            else return q->Add(pg);
        }
        return true;
    }

    //----------------
    int Query(Bound &bound, vector<Point> &query) // Pontok lek�rdez�se 3D befoglal�ra
    {
        int i;
        Point *p;

        if(mPageNum) // lapok vizsg�lata, rekurz�v h�v�s
        {
            for(i = 0; i < mPageNum; i++)
            {
                if(bound.Overlap(mPages[i]->mBound)) mPages[i]->Query(bound, query);
            }
        }
        else // pontok vizsg�lata
        {
            for(i = 0, p = mPoints; i < mPointNum; i++, p++)
            {
                if(bound.Inside(*p))
                    query.push_back(*p);
            }
        }
        return int(query.size());
    }

    //----------------
    void Stat(int &pointNum, int &pageNum, int &level, int pos = 0) // statisztika
    {
        int i;

        for(i = 0; i < pos; i++) printf("> "); // szintek ki�r�sa
        printf("PageNum=%d, PointNum=%d, Level=%d\n", mPageNum, mPointNum, pos);
        //printf("x1=%.2f y1=%.2f x2=%.2f y2=%.2f\n", mBound.x1, mBound.y1, mBound.x2, mBound.y2);

        if(pos > level) level = pos; // szintek sz�ma
        if(mPageNum) // ha page, akkor rekurz�van megh�vjuk a gyerekekre
        {
            pageNum += mPageNum; // lapok sz�ma
            for(i = 0; i < mPageNum; i++) mPages[i]->Stat(pointNum, pageNum, level, pos + 1); // rekurzi�
        }
        else pointNum += mPointNum; // pontok sz�ma

        // �tlagos lapkihaszn�lts�g: page/PageSize �s point/PointSize alapj�n
    }

    //----------------
    void Export(FILE *f, FILE *p, int pos = 0) // BNA ki�r�s
    {
        int i;
        int ptr = int(int8(this)) & 0xFFF; // pointer als� 12 bitje

        // t�glalap ki�r�sa
        fprintf(f, "\"%d\",\"%d\",5\n%.2f,%.2f\n%.2f,%.2f\n%.2f,%.2f\n%.2f,%.2f\n%.2f,%.2f\n",
            pos, ptr, mBound.x1, mBound.y1, mBound.x2, mBound.y1,
            mBound.x2, mBound.y2, mBound.x1, mBound.y2, mBound.x1, mBound.y1);

        if(mPageNum) // ha van page, akkor rekurz�van megh�vjuk a gyerekekre
        {
            for(i = 0; i < mPageNum; i++) mPages[i]->Export(f, p, pos + 1); // rekurzi�
        }
        else
        {
            for(i = 0; i < mPointNum; i++)
            {
                fprintf(p, "\"%d\",\"%d\",1\n%.2f,%.2f\n",
                    i, ptr, mPoints[i].x, mPoints[i].y);
            }
        }
    }

    //----------------
    void Erase() // t�rl�s
    {
        int i;

        if(mPageNum) // ha page, akkor rekurz�van megh�vjuk a gyerekekre
        {
            for(i = 0; i < mPageNum; i++) mPages[i]->Erase(); // rekurzi�
            // gyereklapokat t�r�lni kell, delete mPages[i];
            mPageNum = 0;
        }
        else { delete[] mPoints;  mPoints = 0;  mPointNum = 0; } // pontok t�rl�se
    }
};
