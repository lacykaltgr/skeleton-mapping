// Pontelh� feldolgoz� �s akad�ly felismer� alkalmaz�s - (C) Czimber M�rk 2023
// LAS f�jl form�tum defin�ci�ja

#pragma pack(push,1)

#include <iomanip>

//--------------------------------
typedef unsigned char byte; // unsigned 1 byte integer
typedef unsigned short ushort; // unsigned 2 byte integer
typedef unsigned int uint; // unsigned 4 byte integer
typedef long long int int8; // signed 8 byte integer
typedef unsigned long long int uint8; // signed 8 byte integer

//---------------- LAS Header ----------------

struct LasHead // LAS file header, Arm-on gondot okozhat az eltolt elemek, elemenk�nt kell olvasni
{
	uint	Signature; // LASF, 0x4653414C, offset=0
	ushort	FileSourceID;
	ushort	GlobalEncoding;

	uint	GUID1; // offset=8
	ushort	GUID2;
	ushort	GUID3;

	int8	GUID4; // offset=16

	byte	VersionMajor; // version, offset=24
	byte	VersionMinor;
	char	SystemIdentifier[32];
	char	GeneratingSoftware[32];

	ushort	FileCreationDayOfYear; // offset=90
	ushort	FileCreationYear;
	ushort	HeaderSize; // header 375

	uint	OffsetToPointData; // point offset, offset=96
	uint	NumberOfVariableLengthRecords;

	byte	PointDataRecordFormat; // emiatt cs�szik a strukt�ra 1 b�jtot, ARM bet�lt�s nem lehets�ges, offset=104 (0x68)
	ushort	PointDataRecordLength; // point recsize, // offset=105
	uint	LegacyNumberOfPointRecords; // 0 or pointNumber
	uint	LegacyNumberOfPointsByReturn[5];

	double	XScaleFactor; // X = x*XScaleFactor + XOffset, offset=113
	double	YScaleFactor;
	double	ZScaleFactor;

	double	XOffset; // offset=137
	double	YOffset;
	double	ZOffset;

	double	MaxX; // max coord, offset=161
	double	MinX;
	double	MaxY;

	double	MinY; // min coords, offset=185
	double	MaxZ;
	double	MinZ;

	int8	StartOfWaveformDataPacketRecord; // offset=209
	int8	StartOfFirstExtendedVariableLengthRecord;
	uint	NumberOfExtendedVariableLengthRecords;
	int8	NumberOfPointRecords; // pointNumber
	int8	NumberOfPointsByReturn[15];
};

//---------------- LAS Point ----------------

struct LasPoint // LAS point record, form�tumonk�nt elt�r� kioszt�s, v1..10
{
	int		x;
	int		y;
	int		z;
	ushort	intensity;
	byte	flags; // return number 3, number of return 3, scan direction 1, edge of flight 1
	byte	classification; // low 5 bits: 0=created, 1=unclassified, 2=ground, 3..5=vegetation, 6 building, 7=low point, 8=model key, 9=water

	char	scanAngle;
	byte	userData;
	ushort	pointSourceID;
	double	GPSTime; // GPS id�, pointDataFormat1,3,4,5,6,7,8,9,10 eset�n (nincs 2 eset�n)
	ushort	red, green, blue; // sz�nek, pointDataFormat3,5,7,8,10 eset�n (nincs 1,2,4,6,9 eset�n)
	// 8 �s 10 eset�n van egy NIR jellemz� is, mindig 16 bites, felfel� igaz�tott �rt�k
	int		fill[32];
};


#pragma pack(pop)
