/*
 *   GPS driver to test the functionalities of the Shared transport.
 *   Copyright (C) 2009 Texas Instruments
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef ST_GPS_STUB_H
#define ST_GPS_STUB_H

/* Function prototype for TaskLet-Write function */
void gps_chrdev_tsklet_stub(unsigned long);
void gps_chrdrv_stub_init(void);

/* ST implemented function */
extern void st_int_recv(const unsigned char *data, long count);

#endif /* ST_GPS_STUB_H */
