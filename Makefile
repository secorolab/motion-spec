GENERATED = generated
BUILD = build

SUPPORT_FILES = \
	code-generator/CMakeLists.txt \
	thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.hpp \
	thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.cpp \
	thirdparty/kinova/GEN3_URDF_V12.urdf

define scenario
$(GENERATED)/$(1):
	@mkdir -p $(GENERATED)/$(1)/headers

gen-prepare-$(1): | $(GENERATED)/$(1)
	@cp code-generator/CMakeLists.txt $(GENERATED)/$(1)/CMakeLists.txt
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.hpp $(GENERATED)/$(1)/chainhdsolver_vereshchagin_fext.hpp
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.cpp $(GENERATED)/$(1)/chainhdsolver_vereshchagin_fext.cpp
	@cp thirdparty/kinova/GEN3_URDF_V12.urdf $(GENERATED)/$(1)/GEN3_URDF_V12.urdf

gen-ir-$(1): | $(GENERATED)/$(1)
	@motion-spec-ir-gen models/$(2) -o $(GENERATED)/$(1)/ir.json

gen-code-$(1): gen-ir-$(1)
	@motion-spec-codegen $(GENERATED)/$(1)/ir.json -o $(GENERATED)/$(1)

gen-comp-$(1): gen-prepare-$(1) gen-code-$(1)
	@cmake -S $(GENERATED)/$(1) -B $(GENERATED)/$(1)/build -DCMAKE_BUILD_TYPE=Debug
	@cd $(GENERATED)/$(1)/build && make

$(1): gen-comp-$(1)
endef

$(eval $(call scenario,sc0a,sc0a-right-arm.json))
$(eval $(call scenario,sc0b,sc0b-dual-arm.json))
$(eval $(call scenario,sc1,sc1.json))
$(eval $(call scenario,sc2,sc2.json))


tutorial-html:
	@sphinx-build -M html docs/sphinx/source/ build/

tutorial-live:
	@sphinx-autobuild docs/sphinx/source/ build/

tutorial-pdf:
	@sphinx-build -M latexpdf docs/sphinx/source/ build/


check:
	motion-spec-check models/sc0a-right-arm.json
	motion-spec-check models/sc0b-dual-arm.json
	motion-spec-check models/sc1.json
	motion-spec-check models/sc2.json

count:
	motion-spec-count models/sc0a-right-arm.json
	motion-spec-count models/sc0b-dual-arm.json
	motion-spec-count models/sc1.json
	motion-spec-count models/sc2.json

clean:
	@rm -rf $(GENERATED) $(BUILD)
