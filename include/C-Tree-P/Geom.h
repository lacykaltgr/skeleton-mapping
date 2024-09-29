// Pontelh� feldolgoz� �s akad�ly felismer� alkalmaz�s - (C) Czimber M�rk 2023

const double PI = 3.141592653589793;
const double RAD = PI/180; // �tsz�m�t�s radi�nra
const double DEG = 180/PI; // �tsz�m�t�s fokra

// 3D pont strukt�ra 
struct Point
{
    double x, y, z;
    Point()
    {
        x = y = z = 0;
    }
    Point(double sx, double sy, double sz)
    {
        x = sx;  y = sy;  z = sz;
    }
};

//--------------------------------
// A Bound strukt�ra a val�s befoglal�kat reprezent�lja.
// Tartalmazza a befoglal� x1, x2, y1, �s y2 koordin�t�it.
struct Bound
{
    double x1, x2, y1, y2, z1, z2;

    //----------------
    Bound() // konstruktor
    {
        x1 = x2 = y1 = y2 = z1 = z2 = 0;
    }

    //----------------
    // Befoglal� kiterjeszt�s, ha num==0, akkor �rt�kad�s, egy�bk�nt b?v�t�s
    void Extend(Point &p, int num)
    {
        if(num == 0) // �rt�kad�s
        {
            x1 = x2 = p.x; y1 = y2 = p.y; z1 = z2 = p.z;
        }
        else // b?v�t�s
        {
            if(p.x < x1) x1 = p.x;
            else if(p.x > x2) x2 = p.x;

            if(p.y < y1) y1 = p.y;
            else if(p.y > y2) y2 = p.y;

            if(p.z < z1) z1 = p.z;
            else if(p.z > z2) z2 = p.z;
        }
    }

    //----------------
    // Befoglal� kiterjeszt�s, ha num==0, akkor �rt�kad�s, egy�bk�nt b?v�t�s
    void Extend(Bound &b, int num)
    {
        if(num == 0) // �rt�kad�s
        {
            *this = b;
        }
        else // b?v�t�s
        {
            if(b.x1 < x1) x1 = b.x1;
            if(b.x2 > x2) x2 = b.x2;

            if(b.y1 < y1) y1 = b.y1;
            if(b.y2 > y2) y2 = b.y2;

            if(b.z1 < z1) z1 = b.z1;
            if(b.z2 > z2) z2 = b.z2;
        }
    }

    //----------------
    // a vagy b befoglal�t n�veli jobban c?
    static int ExtendCost(Bound a, Bound b, Bound c)
    {
        double v1, v2, d1, d2;
        v1 = a.Volume();
        a.Extend(c, 1); // a befoglal� b?v�t�se c-vel
        d1 = a.Volume() - v1; // n�veked�s
        v2 = b.Volume();
        b.Extend(c, 1); // b befoglal� b?v�t�se c-vel
        d2 = b.Volume() - v2; // n�veked�s

        if(d1 < d2) return 1;  else return 2; // a n�veked�se kisebb
    }

    //----------------
    bool Inside(Point &p) // Pont a befoglal�n bel�l van
    {
        if(x1 <= p.x && p.x <= x2 && // x,y,z a tartom�nyon bel�l van
            y1 <= p.y && p.y <= y2 &&
            z1 <= p.z && p.z <= z2) return true;
        else return false; // pont k�v�l van
    }

    //----------------
    bool Overlap(Bound &bound)
    {
        if(x1 <= bound.x2 && x2 >= bound.x1 && y1 <= bound.y2 && y2 >= bound.y1) return true; // �tfed
        else return false; // nem fed �t
    }

    //----------------
    double Volume() // befoglal� t�rfogata
    {
        return (x2 - x1) * (y2 - y1) * (z2 - z1);
    }

    //----------------
    void GetMean(Point &pt) // �tlagpont sz�m�t�s
    {
        pt.x = (x1 + x2) * 0.5;
        pt.y = (y1 + y2) * 0.5;
        pt.z = (z1 + z2) * 0.5;
    }
};
