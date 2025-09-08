/* #includes */ /*{{{C}}}*//*{{{*/
//#include "config.h"
#include "fs.h"

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"

#ifdef USE_DMALLOC
#include <dmalloc.h>
#endif
/*}}}*/

#define FR_OK 0

/* Device_open           -- Open an image file                      */ /*{{{*/
const char *Device_open(struct Device *this, const char *filename, int mode, const char *deviceOpts)
{
  strcpy(buf, filename);
  this->fd = fs_open();
  this->opened = this->fd == FR_OK;
 
  return ((this->fd != FR_OK ) ? "Error open file" : (const char *)0);
}
/*}}}*/
/* Device_setGeometry    -- Set disk geometry                       */ /*{{{*/
void Device_setGeometry(struct Device *this, int secLength, int sectrk, int tracks, off_t offset)
{
  this->secLength=secLength;
  this->sectrk=sectrk;
  this->tracks=tracks;
  this->offset=offset;
}
/*}}}*/
/* Device_close          -- Close an image file                     */ /*{{{*/
const char *Device_close(struct Device *this)
{
  this->opened=0;
  return (const char*)0;
}
/*}}}*/
/* Device_readSector     -- read a physical sector                  */ /*{{{*/
const char *Device_readSector(const struct Device *this, int track, int sector, char *buf1)
{
  int res;

  assert(this);
  assert(sector>=0);
  assert(sector<this->sectrk);
  assert(track>=0);
  assert(track<this->tracks);
  assert(buf1);
  if (fs_lseek((off_t)(((sector+track*this->sectrk)*this->secLength)+this->offset),SEEK_SET)!=FR_OK) 
  {
    return "Error image seek";
  }
  if ((res=fs_read0(buf1, this->secLength)) != FR_OK) 
  {
    //if (res==-1)
    {
      return "Error image read"; //strerror(errno);
    }
    //else memset(buf+res,0,this->secLength-res); /* hit end of disk image */
  }
  return (const char*)0;
}
/*}}}*/
/* Device_writeSector    -- write physical sector                   */ /*{{{*/
const char *Device_writeSector(const struct Device *this, int track, int sector, const char *buf1)
{
  assert(sector>=0);
  assert(sector<this->sectrk);
  assert(track>=0);
  assert(track<this->tracks);
  if (fs_lseek((off_t)(((sector+track*this->sectrk)*this->secLength)+this->offset),SEEK_SET)!=FR_OK) 
  {
    return "Error image seek";
  }
  fs_wtotal = this->secLength;
  if (fs_write_start() != FR_OK)
    return "Error write to image";
  fs_file_wlen = this->secLength;
  memcpy(fs_file_wbuf, buf1, this->secLength);
  if (fs_write_end() != FR_OK)
    return "Error write to image";
  
  return (const char*)0;
}
/*}}}*/
