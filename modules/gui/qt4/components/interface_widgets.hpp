/*****************************************************************************
 * interface_widgets.hpp : Custom widgets for the main interface
 ****************************************************************************
 * Copyright (C) 2006 the VideoLAN team
 * $Id$
 *
 * Authors: Clément Stenac <zorglub@videolan.org>
 *          Jean-Baptiste Kempf <jb@videolan.org>
 *          Rafaël Carré <funman@videolanorg>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifndef _INTFWIDGETS_H_
#define _INTFWIDGETS_H_

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_interface.h>
#include <vlc_aout.h>

#include "qt4.hpp"
#include "main_interface.hpp"
#include "input_manager.hpp"

#include <QWidget>
#include <QFrame>

#define VOLUME_MAX 200

/* on WIN32 hide() for fullscreen controller doesnt work, so it have to be
   done by trick with setting the opacity of window */
#ifdef WIN32
    #define WIN32TRICK
#endif

/* to trying transparency with fullscreen controller on windows enable that */
#define HAVE_TRANSPARENCY 0
/* it can be enabled on-non windows systems,
   but it will be transparent only with composite manager */
#ifndef WIN32
    #define HAVE_TRANSPARENCY 1
#endif

class ResizeEvent;
class QPalette;
class QPixmap;
class QLabel;
class QHBoxLayout;

/******************** Video Widget ****************/
class VideoWidget : public QFrame
{
    Q_OBJECT
friend class MainInterface;

public:
    VideoWidget( intf_thread_t * );
    virtual ~VideoWidget();

    void *request( vout_thread_t *, int *, int *,
                   unsigned int *, unsigned int * );
    void  release( void * );
    int   control( void *, int, va_list );

    virtual QSize sizeHint() const;
private:
    intf_thread_t *p_intf;
    vout_thread_t *p_vout;

    vlc_mutex_t lock;
    QSize videoSize;

signals:
    void askVideoWidgetToShow();
    //void askResize();

public slots:
    void SetSizing( unsigned int, unsigned int );
};

/******************** Background Widget ****************/
class BackgroundWidget : public QWidget
{
    Q_OBJECT
public:
    BackgroundWidget( intf_thread_t * );
    virtual ~BackgroundWidget();

private:
    QPalette plt;
    QLabel *label;
    virtual void contextMenuEvent( QContextMenuEvent *event );
    intf_thread_t *p_intf;
    virtual void resizeEvent( QResizeEvent * event );
public slots:
    void toggle(){ TOGGLEV( this ); }
    void updateArt( QString );
};

class VisualSelector : public QFrame
{
    Q_OBJECT
public:
    VisualSelector( intf_thread_t *);
    virtual ~VisualSelector();
private:
    intf_thread_t *p_intf;
    QLabel *current;
private slots:
    void prev();
    void next();
};

/* Advanced Button Bar */
class QPushButton;
class AdvControlsWidget : public QFrame
{
    Q_OBJECT
public:
    AdvControlsWidget( intf_thread_t *);
    virtual ~AdvControlsWidget();

    void enableInput( bool );
    void enableVideo( bool );

private:
    intf_thread_t *p_intf;
    QPushButton *recordButton, *ABButton;
    QPushButton *snapshotButton, *frameButton;

    mtime_t timeA, timeB;

private slots:
    void snapshot();
#if 0
    void frame();
#endif
    void fromAtoB();
    void record();
    void AtoBLoop( float, int, int );
};

/* Button Bar */
class InputSlider;
class QSlider;
class QGridLayout;
class VolumeClickHandler;
class SoundSlider;
class QAbstractSlider;
class QToolButton;

class ControlsWidget : public QFrame
{
    Q_OBJECT
public:
    /* p_intf, advanced control visible or not, blingbling or not */
    ControlsWidget( intf_thread_t *_p_i, MainInterface *_p_mi,
        bool b_advControls, bool b_shiny, bool b_fsCreation = false);
    virtual ~ControlsWidget();

    QPushButton *playlistButton;
    void setStatus( int );
    void enableInput( bool );
    void enableVideo( bool );
public slots:
    void setNavigation( int );
protected:
    friend class MainInterface;
    friend class VolumeClickHandler;
protected:
    intf_thread_t       *p_intf;
    QWidget             *discFrame;
    QWidget             *telexFrame;
    QGridLayout         *controlLayout;
    InputSlider         *slider;
    QPushButton         *prevSectionButton, *nextSectionButton, *menuButton;
    QPushButton         *playButton, *fullscreenButton, *extSettingsButton;
    QToolButton         *slowerButton, *fasterButton;
    QHBoxLayout         *controlButLayout;
    AdvControlsWidget   *advControls;
    QLabel              *volMuteLabel;
    QAbstractSlider     *volumeSlider;
    VolumeClickHandler  *hVolLabel;

    bool                 b_advancedVisible;
protected slots:
    void play();
    void stop();
    void prev();
    void next();
    void updateVolume( int );
    void updateVolume( void );
    void updateInput();
    void fullscreen();
    void extSettings();
    void faster();
    void slower();
    void toggleAdvanced();
signals:
    void advancedControlsToggled( bool );
};

/***********************************
 * Fullscreen controller
 ***********************************/

static int showFullscreenControllCallback(vlc_object_t *vlc_object, const char *variable, vlc_value_t old_val,
    vlc_value_t new_val, void *data);

static int regMouseMoveCallback(vlc_object_t *vlc_object, const char *variable, vlc_value_t old_val,
    vlc_value_t new_val, void *data);

class FullscreenControllerWidget : public ControlsWidget
{
    Q_OBJECT
public:
    FullscreenControllerWidget( intf_thread_t *, MainInterface*, bool, bool );
    virtual ~FullscreenControllerWidget();

    void SetHideTimeout( int hideTimeout ) { i_hideTimeout = hideTimeout; }
    void regFullscreenCallback( vout_thread_t *p_vout );

    bool isFSCHidden();

public slots:
    void unregFullscreenCallback();

protected:
    friend class MainInterface;
    friend class VolumeClickHandler;

    virtual void mouseMoveEvent( QMouseEvent *event );
    virtual void mousePressEvent( QMouseEvent *event );
    virtual void enterEvent( QEvent *event );
    virtual void leaveEvent( QEvent *event );
    virtual void keyPressEvent( QKeyEvent *event );

private slots:
    void hideFSControllerWidget();
    void slowHideFSC();

private:
    QTimer *p_hideTimer;

#if HAVE_TRANSPARENCY
    QTimer *p_slowHideTimer;
#endif

    int i_lastPosX;
    int i_lastPosY;
    int i_hideTimeout;  /* FSC hiding timeout, same as mouse hiding timeout */
    bool b_mouseIsOver;

#ifdef WIN32TRICK
    bool fscHidden;
#endif

    virtual void customEvent( QEvent *event );
};



class VolumeClickHandler : public QObject
{
public:
    VolumeClickHandler( intf_thread_t *_p_intf, ControlsWidget *_m ) :QObject(_m)
    {m = _m; p_intf = _p_intf; }
    virtual ~VolumeClickHandler() {};
    bool eventFilter( QObject *obj, QEvent *e )
    {
        if (e->type() == QEvent::MouseButtonPress  )
        {
            aout_VolumeMute( p_intf, NULL );
            audio_volume_t i_volume;
            aout_VolumeGet( p_intf, &i_volume );
            m->updateVolume( i_volume *  VOLUME_MAX / (AOUT_VOLUME_MAX/2) );
            return true;
        }
        return false;
    }
private:
    ControlsWidget *m;
    intf_thread_t *p_intf;
};

#include <QLabel>
#include <QMouseEvent>
class TimeLabel : public QLabel
{
    Q_OBJECT
    void mousePressEvent( QMouseEvent *event )
    {
        emit timeLabelClicked();
    }
    void mouseDoubleClickEvent( QMouseEvent *event )
    {
        emit timeLabelDoubleClicked();
    }
signals:
    void timeLabelClicked();
    void timeLabelDoubleClicked();
};

class SpeedLabel : public QLabel
{
    Q_OBJECT
public:
    SpeedLabel( intf_thread_t *_p_intf, const QString text ): QLabel( text)
    { p_intf = _p_intf; }

protected:
    virtual void mouseDoubleClickEvent ( QMouseEvent * event )
    {
        THEMIM->getIM()->setRate( INPUT_RATE_DEFAULT );
    }
private:
    intf_thread_t *p_intf;
};

/******************** Speed Control Widgets ****************/
class SpeedControlWidget : public QFrame
{
    Q_OBJECT
public:
    SpeedControlWidget( intf_thread_t *);
    virtual ~SpeedControlWidget();
    void updateControls( int );
private:
    intf_thread_t *p_intf;
    QSlider *speedSlider;
public slots:
    void setEnable( bool );
private slots:
    void updateRate( int );
    void resetRate();
};

#endif
