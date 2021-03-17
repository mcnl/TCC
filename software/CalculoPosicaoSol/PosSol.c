#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>


const double PI  =  3.14159265358979323846;
const double rad =  PI / 180.0;
const double e   =  rad * 23.4397;

double roundRadians(double beta){
	double trash;
	if(beta>2*PI){
		beta = beta/2*PI;
		beta = modf(beta, &trash);
	}
	return beta;
}


double dateToMiliseconds(int sec,int minute,int hour,int day, int month, int year){
	struct tm date = {
		.tm_sec=sec,
		.tm_min=minute,
		.tm_hour=hour,
		.tm_mday=day,
		.tm_mon=month,
		.tm_year=year - 1900
	};
	return ((double)mktime(&date))*1000.0;
}

double ToJulian(double date){
	return (date - dateToMiliseconds(1,0,0,1,1,1970)) / (1000.0 * 60.0 * 60.0 * 24.0) - 0.5 + 2440588.0;
}

double ToDays(double date){
	return ToJulian(date) - 2451545.0;
}

double rightAscencion(double d){
	double m = rad * (357.5291 + 0.98560028 * d);
	double c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	double p = rad * 102.9372; 
	double l = m + c + p + PI;
	double ra = atan2((sin(l) * cos(e) - tan(0) * sin(e)),cos(l));
	
	return ra;
}
double declination(double d){
	double m = rad * (357.5291 + 0.98560028 * d);
	double c = rad * (1.9148 * sin(m) + 0.02 * sin(2 * m) + 0.0003 * sin(3 * m));
	double p = rad * 102.9372; 
	double l = m + c + p + PI;
	double dec = asin(sin(0) * cos(e) + cos(0) * sin(e) * sin(l));
	return dec;
}

double sideralTime(double d, double lw){
	return rad * (280.16 + 360.9856235 * d) - lw;
}

double azimuth(double H, double phi, double dec){
	return PI + atan2(sin(H), (cos(H) * sin(phi) - tan(dec) * cos(phi)));
}

double altitude(double H, double phi, double dec){
	return PI + asin(roundRadians(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H)));
}

double* getposition(double date, double lat, double lng){
	double lw = rad * (-lng);
	double phi = rad * lat;
	double d = ToDays(date);
	double RA = rightAscencion(d);
	double DEC = declination(d);
	double H = sideralTime(d,lw) - RA;

	double az = azimuth(H,phi,DEC);
	double alt = altitude(H,phi,DEC);

	static double result[2];
	result[0] = 360 - (az)*180.0/PI;
	result[1] = (alt)*180.0/PI - 180;
	return result;
} 

int main(){
	printf("Sun Position Software - Writen by Matheus Casa Nova da Luz\n");
	double *ret;
	double now;
	for(int i = 0; i<24;i++){
		now = (double)dateToMiliseconds(0,0,(24 - i)+3,10,3,2021); 
		ret = getposition(now,-8.15762000,-34.91477200);
		if(*(ret + 1)!=0){
			printf("Time: %d\n Elevacao: %f Azimute: %f\n\n", i, *(ret + 1), *(ret + 0));	
		}
	}
	
	return 0;
}