/**
 * @brief 
 * 
 * @file server_example_main.cpp
 * 
 * @author Igor Makhtes (igor@cogniteam.com)
 * @date 2020-05-05
 * @copyright Cogniteam (c) 2020
 *    
 * MIT License
 *   
 * Permission is hereby granted, free of charge, to any person obtaining a  copy
 * of this software and associated documentation files (the 'Software'), to deal
 * in the Software without restriction, including without  limitation the rights
 * to use, copy, modify, merge,  publish,  distribute,  sublicense,  and/or sell
 * copies of the Software, and  to  permit  persons  to  whom  the  Software  is 
 * furnished to do so, subject to the following conditions:
 *   
 * The   above   copyright   notice   and  this   permission   notice  shall  be
 * included in all copies or substantial portions of the Software.
 *   
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY  KIND,  EXPRESS  OR
 * IMPLIED, INCLUDING BUT NOT LIMITED  TO  THE  WARRANTIES  OF  MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN  NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE  LIABLE  FOR  ANY  CLAIM,  DAMAGES  OR  OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */


#include <Ros2ActionWaitServer.h>


/**
 * @brief 
 * @param argv argv[1] is the name of the action (ros2 runner)
 */
int main(int argc, char ** argv) {

    using namespace std;

    if (argc < 2) {
        cerr << "Error: Missing action name" << endl;
        cout << "Usage:" << argv[0] << " ACTION_NAME" << endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = nullptr;

    node = rclcpp::Node::make_shared(argv[1]);

    auto action_server = std::make_shared<Ros2ActionWaitServer>(
            node, argv[1]);

    cout << "ROS2 action server '" << argv[1] << "' started" << endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}