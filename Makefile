GEN = gen
BUILD = build

$(GEN):
	@mkdir -p $@

gen-prepare: | $(GEN)
	@cp code-generator/CMakeLists.txt $(GEN)/CMakeLists.txt
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.hpp $(GEN)/chainhdsolver_vereshchagin_fext.hpp
	@cp thirdparty/orocos-kdl/chainhdsolver_vereshchagin_fext.cpp $(GEN)/chainhdsolver_vereshchagin_fext.cpp
	@cp thirdparty/kinova/GEN3_URDF_V12.urdf $(GEN)/GEN3_URDF_V12.urdf

gen-code:
	@motion-spec-codegen $(GEN)/ir.json -o $(GEN)

gen-comp:
	@cmake -S $(GEN) -B $(GEN)/build -DCMAKE_BUILD_TYPE=Debug
	@cd $(GEN)/build && make

gen-ir-sc0a:
	@motion-spec-ir-gen models/sc0a-right-arm.json -o $(GEN)/ir.json

gen-ir-sc0b:
	@motion-spec-ir-gen models/sc0b-dual-arm.json -o $(GEN)/ir.json

gen-ir-sc1:
	@motion-spec-ir-gen models/sc1.json -o $(GEN)/ir.json

gen-ir-sc2:
	@motion-spec-ir-gen models/sc2.json -o $(GEN)/ir.json

sc0a: gen-prepare gen-ir-sc0a gen-code gen-comp
sc0b: gen-prepare gen-ir-sc0b gen-code gen-comp
sc1: gen-prepare gen-ir-sc1 gen-code gen-comp
sc2: gen-prepare gen-ir-sc2 gen-code gen-comp


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
	@rm -rf $(GEN) $(BUILD)
