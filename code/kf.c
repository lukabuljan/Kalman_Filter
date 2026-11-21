#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>

#include "funk.h"

int main(void)
{
	char *imena[] = {"../data/accelerometer.csv","../data/attitude.csv","../data/barometer.csv", "../data/estimated_state.csv"};

	FILE *akc_file = fopen(imena[0], "r"); 		//Ucitvanje datoteka, 3 za čitanje, 1 za pisanje
	FILE *att_file = fopen(imena[1], "r");
	FILE *bar_file = fopen(imena[2], "r");
	FILE *est_file = fopen(imena[3], "w");

	if (att_file == NULL){printf("Greska u otvaranju datoteke.\n"); return 1;}
	char header[128];

	if (fgets(header, sizeof(header), att_file) == NULL){printf("Greska u zaglavlju.\n"); fclose(att_file); return 1;} //Preskakanje zaglavlja

	uint32_t n = 0; //brojač za kretanje po redovima datoteke accelerometer.csv

	acc att_prvi;   //varijable za ucitavanje podataka iz attitude.csv
	acc att_drugi;
	acc att_inter;
	int read_att;

	acc acc_prvi;  //varijable za ucitavanje podataka iz accelerometer.csv
	acc_prvi.time = 0;
	acc_prvi.x = 0;
	acc_prvi.y = 0;
	acc_prvi.z = 0;

	bar bar_prvi; //varijable za ucitavanje podataka iz barometer.csv
	bar bar_drugi;
	uint8_t bar_eof = 0;

	acc fix;	//varijable za pohranu preračunatih akceleracija drona u fiksni k.s.
	
	uint32_t t2;	//varijabla za pohranu vremena

	if (fgets(header, sizeof(header), bar_file) == NULL){printf("Greska u zaglavlju.\n"); fclose(bar_file); return 1;} //preskakakanje zaglavlja
	int read_bar = fscanf(bar_file, "%d, %f", &bar_prvi.time, &bar_prvi.z);
	if (read_bar == EOF)return 1;
	if (read_bar != 2){printf("Greska u formatiranju.\n"); fclose(bar_file); free(bar_file); return 1;}
	
	kalman s;	//varijabla za pohranu podataka vezanih u Kalamanov filtar
	kalman_init(&s, &bar_prvi);
	uint8_t acc_bar;

	estimate result;//varijabla za pohranu estimacije
	fprintf(est_file, "sys_time_us,est_altitude,est_velocity\n"); //ispis zaglavlja

	do 
	{
		if(!n)  //ucitavanje prva dva reda iz attitude.csv, ujedno citanje accelerometer.csv i preskakanje zaglavlja.
		{
			read_att = fscanf(att_file, "%" SCNu32 ", %f, %f, %f", &att_prvi.time, &att_prvi.x, &att_prvi.y, &att_prvi.z);
			if (read_att == EOF)return 1;
			if (read_att != 4){printf("Greska u formatiranju.\n"); fclose(att_file); free(att_file); return 1;}

			read_att = fscanf(att_file, "%" SCNu32 ", %f, %f, %f", &att_drugi.time, &att_drugi.x, &att_drugi.y, &att_drugi.z);
		
			if (fgets(header, sizeof(header), akc_file) == NULL){printf("Greska u zaglavlju.\n"); fclose(akc_file); return 1;}
			if (read_att == EOF)return 1;
			if (read_att != 4){printf("Greska u formatiranju.\n"); fclose(att_file); free(att_file); return 1;}
		}
		else	//za sve preostale podatke iz attitude .csv drugi podatak postaji prvi iz proslog ucitavanja, a drugi se nanovo ucita
		{
			att_prvi = att_drugi;

			read_att = fscanf(att_file, "%" SCNu32 ", %f, %f, %f", &att_drugi.time, &att_drugi.x, &att_drugi.y, &att_drugi.z);
			if (read_att == EOF)return 1;
			if (read_att != 4){printf("Greska u formatiranju.\n"); fclose(att_file); free(att_file); return 1;}
		}

		while (acc_prvi.time < att_prvi.time) //Redovi iz accelerometer.csv se ucitavaju dok se ne nade mjerenje unutar trenutnog intervala attitude.csv
		{	
			int read_acc = fscanf(akc_file, "%" SCNu32 ", %f, %f, %f", &acc_prvi.time, &acc_prvi.x, &acc_prvi.y, &acc_prvi.z);
			if (read_acc == EOF)return 1;
			if (read_acc != 4){printf("Greska u formatiranju.\n"); fclose(akc_file); free(akc_file); return 1;}
		}

		while (acc_prvi.time <= att_drugi.time) //Ucitavanje redova između dva attitude.csv podatka i interpolacija istih. . 
		{
			if(acc_prvi.time <= bar_drugi.time && !bar_eof) //Ako je sljedeci podatak mjerenje iz accelerometer.csv izvrši predikciju Kalmanovog filtra i ispisi rezultat u datoteku estimated_values.csv
			{
				acc_bar = 1;
				t2 = acc_prvi.time;

				interpol(acc_prvi.time, &att_prvi, &att_drugi, &att_inter);
				acc fix = mob_to_fix(acc_prvi, att_inter);
				kalman_filter(&s, t2, bar_drugi.z, &fix, acc_bar, &result);
				fprintf(est_file,"%" PRIu32 ",%.6f,%.6f\n", result.time, result.z, result.vz);
				
				int read_acc = fscanf(akc_file, "%" SCNu32 ", %f, %f, %f", &acc_prvi.time, &acc_prvi.x, &acc_prvi.y, &acc_prvi.z);
				if (read_acc == EOF)return 1;
				if (read_acc != 4){printf("Greska u formatiranju.\n"); fclose(akc_file); free(akc_file); return 1;}
			}
			else if(acc_prvi.time > bar_drugi.time && !bar_eof) //Ako je sljedeci podatak mjerenje iz barometer.csv izvrši korekciju Kalmanovog filtra
			{
				acc_bar = 0; 
				t2 = bar_drugi.time;

				kalman_filter(&s, t2, bar_drugi.z, &fix, acc_bar, &result);
				read_bar = fscanf(bar_file, "%" SCNu32 ", %f", &bar_drugi.time, &bar_drugi.z); //ucitaj sljedece mjerenje
				if (read_bar == EOF){bar_eof = 1;} //varijabla koja označava da su svi podaci iz barometer.csv ucitani
			}
			else if(bar_eof == 1) //ako su ucitana sva mjerenja iz barometer.csv izvrši predikciju s preostalim podacima iz accelerometer.csv
			{
				acc_bar = 1;
				t2 = acc_prvi.time;

				interpol(acc_prvi.time, &att_prvi, &att_drugi, &att_inter);
				acc fix = mob_to_fix(acc_prvi, att_inter);
				kalman_filter(&s, t2, bar_drugi.z, &fix, acc_bar, &result);
				fprintf(est_file,"%" PRIu32 ",%.6f,%.6f\n", result.time, result.z, result.vz); //pisanje u estimated_values.csv
				
				int read_acc = fscanf(akc_file, "%" SCNu32 ", %f, %f, %f", &acc_prvi.time, &acc_prvi.x, &acc_prvi.y, &acc_prvi.z);
				if (read_acc == EOF)return 1;
				if (read_acc != 4){printf("Greska u formatiranju.\n"); fclose(akc_file); free(akc_file); return 1;}
			}
		}


		n++;

	}while(!feof(akc_file));
	
	return 0;
}
