#ifndef CARDREADER_H
#define CARDREADER_H

#ifdef SDSUPPORT

#include "SdFile.h"
enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};
class CardReader
{
	private:
		SdFile root,*curDir,workDir,workDirParent,workDirParentParent;
		Sd2Card card;
		SdVolume volume;
		SdFile file;
		uint32_t filesize;
		//int16_t n;
		unsigned long autostart_atmillis;
		uint32_t sdpos ;
		unsigned long volsize;
		bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

		LsAction lsAction; //stored for recursion.
		int16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
		char* diveDirName;
		void lsDive(const char *prepend,SdFile parent);

public:
  CardReader();
  
  void initsd();
  void write_command(char *buf);
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

  void checkautostart(bool x); 
  void openFile(char* name,bool read);
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile();
  void release();
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished();

  void getfilename(const uint8_t nr);
  uint16_t getnrfilenames();
  

  void ls();
  void chdir(const char * relpath);
  void updir();
  void setroot();
  unsigned long Volsize() const { return volsize; }


  bool isFileOpen();
  bool eof();;
  int16_t get();;
  void setIndex(long index);;
  float percentDone();;
  char* getWorkDirName();;

public:
  bool saving;
  bool logging;
  bool sdprinting ;  
  bool cardOK ;
  char filename[13];
  char longFilename[LONG_FILENAME_LENGTH];
  bool filenameIsDir;
  int lastnr; //last number of the autostart;

};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting)

#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define IS_SD_INSERTED (READ(SDCARDDETECT)!=0)
# else
#  define IS_SD_INSERTED (READ(SDCARDDETECT)==0)
# endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
# define IS_SD_INSERTED true
#endif

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT
#endif
