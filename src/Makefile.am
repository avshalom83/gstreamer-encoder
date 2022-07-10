lib_LTLIBRARIES=encoder_gstreamer.la

CPPFLAGS+=$(GS_CFLAGS) $(GSBASE_CFLAGS) $(GSVIDEO_CFLAGS) $(GSAPP_CFLAGS)
LDFLAGS+=$(GS_LIBS) $(GSBASE_LIBS) $(GSVIDEO_LIBS) $(GSAPP_LIBS)

encoder_gstreamer_la_LDFLAGS = -module -avoid-version
encoder_gstreamer_la_SOURCES = encoder-interface.cpp
