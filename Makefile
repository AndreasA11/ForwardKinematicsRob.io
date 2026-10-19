.PHONY: build run demo demo-portfolio portfolio portfolio-video viewer clean test test-tcp
PUGIXML := third_party/pugixml
build: main build/urdf_example

# *CHANGES*: Keep make run fast after build by tracking communication source dependencies.
# *CHANGES*: Rebuild the runtime when the Task 2 parameter services change.
# *CHANGES*: Link the demo servo and FSM while keeping their startup behind --demo.
main: src/main.cpp src/robot_model.cpp src/math3d.cpp src/forward_kinematics.cpp src/robot_description_loader.cpp src/robot_world_state_publisher.cpp src/joint_state_publisher.cpp src/finite_state_machine.cpp include/robot_model.hpp include/math3d.hpp include/forward_kinematics.hpp include/robot_description_loader.hpp include/robot_world_state_publisher.hpp include/joint_state_publisher.hpp include/finite_state_machine.hpp include/parameter_server.hpp include/middleware/middleware.hpp include/middleware/rosbridge_server.hpp include/middleware/json.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	g++ -std=c++20 -O2 -pthread -Iinclude -I$(PUGIXML) -o main src/main.cpp src/robot_model.cpp src/math3d.cpp src/forward_kinematics.cpp src/robot_description_loader.cpp src/robot_world_state_publisher.cpp src/joint_state_publisher.cpp src/finite_state_machine.cpp $(PUGIXML)/pugixml.cpp

build/urdf_example: src/urdf_example.cpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -I$(PUGIXML) -o build/urdf_example src/urdf_example.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Give the parser a direct verification target without starting runtime nodes.
build/robot_model_test: tests/robot_model_test.cpp src/robot_model.cpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -Iinclude -I$(PUGIXML) -o build/robot_model_test tests/robot_model_test.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Verify matrix conventions and difficult quaternion angles separately.
build/math3d_test: tests/math3d_test.cpp src/math3d.cpp include/math3d.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -Iinclude -o build/math3d_test tests/math3d_test.cpp src/math3d.cpp

# *CHANGES*: Exercise Task 5 FK through the parsed model, without runtime services.
build/forward_kinematics_test: tests/forward_kinematics_test.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp include/forward_kinematics.hpp include/math3d.hpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -Iinclude -I$(PUGIXML) -o build/forward_kinematics_test tests/forward_kinematics_test.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Verify Task 6 service polling, status timing, and atomic replacement.
build/robot_description_loader_test: tests/robot_description_loader_test.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp include/robot_description_loader.hpp include/parameter_server.hpp include/forward_kinematics.hpp include/math3d.hpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -I$(PUGIXML) -o build/robot_description_loader_test tests/robot_description_loader_test.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Check Task 7 joint-state and periodic `/tf` behavior through middleware.
build/robot_state_publisher_test: tests/robot_state_publisher_test.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp include/robot_description_loader.hpp include/parameter_server.hpp include/forward_kinematics.hpp include/math3d.hpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -I$(PUGIXML) -o build/robot_state_publisher_test tests/robot_state_publisher_test.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Verify Task 8 lifecycle, timing, and world-tree replacement in process.
build/topic_lifecycle_test: tests/topic_lifecycle_test.cpp src/robot_world_state_publisher.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp include/robot_world_state_publisher.hpp include/robot_description_loader.hpp include/parameter_server.hpp include/forward_kinematics.hpp include/math3d.hpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -I$(PUGIXML) -o build/topic_lifecycle_test tests/topic_lifecycle_test.cpp src/robot_world_state_publisher.cpp src/robot_description_loader.cpp src/forward_kinematics.cpp src/math3d.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Verify Task 9 global-pose composition and latest-tree behavior directly.
build/world_state_publisher_test: tests/world_state_publisher_test.cpp src/robot_world_state_publisher.cpp src/math3d.cpp include/robot_world_state_publisher.hpp include/math3d.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -o build/world_state_publisher_test tests/world_state_publisher_test.cpp src/robot_world_state_publisher.cpp src/math3d.cpp

# *CHANGES*: Verify the demo servo's target, reset, and reload behavior through middleware.
build/joint_state_publisher_test: tests/joint_state_publisher_test.cpp src/joint_state_publisher.cpp src/robot_model.cpp include/joint_state_publisher.hpp include/robot_model.hpp $(PUGIXML)/pugixml.cpp $(PUGIXML)/pugixml.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -I$(PUGIXML) -o build/joint_state_publisher_test tests/joint_state_publisher_test.cpp src/joint_state_publisher.cpp src/robot_model.cpp $(PUGIXML)/pugixml.cpp

# *CHANGES*: Verify Task 11 FSM validation, state transitions, arrival tolerance, and storage.
build/finite_state_machine_test: tests/finite_state_machine_test.cpp src/finite_state_machine.cpp include/finite_state_machine.hpp
	mkdir -p build
	g++ -std=c++20 -O2 -pthread -Iinclude -o build/finite_state_machine_test tests/finite_state_machine_test.cpp src/finite_state_machine.cpp

# *CHANGES*: Include Task 10's direct servo test and Task 11's FSM test in the routine suite.
test: build/robot_model_test build/math3d_test build/forward_kinematics_test build/robot_description_loader_test build/robot_state_publisher_test build/topic_lifecycle_test build/world_state_publisher_test build/joint_state_publisher_test build/finite_state_machine_test
	./build/robot_model_test
	./build/math3d_test
	./build/forward_kinematics_test
	./build/robot_description_loader_test
	./build/robot_state_publisher_test
	./build/topic_lifecycle_test
	./build/world_state_publisher_test
	./build/joint_state_publisher_test
	./build/finite_state_machine_test

# *CHANGES*: Keep the external TCP lifecycle check separate from offline unit tests.
test-tcp: build
	python3 tests/runtime_topic_tcp.py
run: main
	./main
# *CHANGES*: Start Task 10's servo only for the portfolio demo, or launch the interactive portfolio demo if requested.
demo: main
ifdef PORTFOLIO
	python3 scripts/demo_portfolio.py
else
	./main --demo
endif
# *CHANGES*: Launch the complete interactive Chess Arm portfolio demo with live runtime, viewer, and browser.
demo-portfolio: build
	python3 scripts/demo_portfolio.py
portfolio: demo-portfolio
# *CHANGES*: Render and compile the high-definition choreography demonstration video and poster.
portfolio-video: build
	python3 scripts/generate_portfolio_video.py
# *CHANGES*: Launch Task 13 external 3D robot viewer and portfolio server.
viewer:
	python3 scripts/viewer.py
clean:
	rm -rf main build
