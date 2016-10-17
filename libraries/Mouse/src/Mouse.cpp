/*
  Mouse.cpp

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Mouse.h"
#include "usbd_hid.h"
#include "usbd_desc.h"

#if defined(USBCON)

static USBD_HandleTypeDef g_USBD_Device;

//================================================================================
//================================================================================
//	Mouse

Mouse_::Mouse_(void) : _buttons(0)
{
}

void Mouse_::begin(void)
{
  /* Init Device Library */
  USBD_Init(&g_USBD_Device, &HID_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&g_USBD_Device, USBD_HID_CLASS);

  /* Start Device Process */
  USBD_Start(&g_USBD_Device);
}

void Mouse_::end(void)
{
  /* Stop Device Process */
  USBD_Stop(&g_USBD_Device);

  /* Deinit Device Library */
  USBD_DeInit(&g_USBD_Device);
}

void Mouse_::click(uint8_t b)
{
	_buttons = b;
	move(0,0,0);
	_buttons = 0;
	move(0,0,0);
}

void Mouse_::move(signed char x, signed char y, signed char wheel)
{
	uint8_t m[4];
	m[0] = _buttons;
	m[1] = x;
	m[2] = y;
	m[3] = wheel;

  USBD_HID_SendReport(&g_USBD_Device, m, 4);
}

void Mouse_::buttons(uint8_t b)
{
	if (b != _buttons)
	{
		_buttons = b;
		move(0,0,0);
	}
}

void Mouse_::press(uint8_t b)
{
	buttons(_buttons | b);
}

void Mouse_::release(uint8_t b)
{
	buttons(_buttons & ~b);
}

bool Mouse_::isPressed(uint8_t b)
{
	if ((b & _buttons) > 0)
		return true;
	return false;
}

Mouse_ Mouse;

#endif
