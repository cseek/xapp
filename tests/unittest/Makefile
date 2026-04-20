CXX      = g++
CXXFLAGS = -I../../src/proto_parser -std=c++17
TESTDIR  = .
BINDIR   = bin

TARGETS = test_main test_rawimu test_gga test_gsv test_rmc test_gst test_gsa \
          test_bestgnssposa test_dualantennaheadinga test_inspvaa test_insstdevsa \
          test_rawimub test_bestgnssposb test_dualantennaheadingb test_inspvab test_insstdevsb

all: $(BINDIR) $(addprefix $(BINDIR)/,$(TARGETS))

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/test_main: $(TESTDIR)/test_main.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_rawimu: $(TESTDIR)/test_rawimua.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_gga: $(TESTDIR)/test_gga.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_gsv: $(TESTDIR)/test_gsv.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_rmc: $(TESTDIR)/test_rmc.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_gst: $(TESTDIR)/test_gst.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_gsa: $(TESTDIR)/test_gsa.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_bestgnssposa: $(TESTDIR)/test_bestgnssposa.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_dualantennaheadinga: $(TESTDIR)/test_dualantennaheadinga.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_inspvaa: $(TESTDIR)/test_inspvaa.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_insstdevsa: $(TESTDIR)/test_insstdevsa.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_rawimub: $(TESTDIR)/test_rawimub.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_bestgnssposb: $(TESTDIR)/test_bestgnssposb.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_dualantennaheadingb: $(TESTDIR)/test_dualantennaheadingb.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_inspvab: $(TESTDIR)/test_inspvab.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

$(BINDIR)/test_insstdevsb: $(TESTDIR)/test_insstdevsb.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -rf $(BINDIR)