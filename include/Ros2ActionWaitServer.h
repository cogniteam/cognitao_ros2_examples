/**
 * @brief ROS 2Action Wait Server
 * 
 * @file Ros2ActionWaitServer.h
 * 
 * @author Lin Azan (lin@cogniteam.com)
 * @date 2020-03-15
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


#ifndef INCLUDE_ROS2_ACTION_WAIT_SERVER_H_
#define INCLUDE_ROS2_ACTION_WAIT_SERVER_H_


#include <inttypes.h>
#include <memory>

#include <cognitao_ros2/action/action.hpp>
#include <cognitao_ros2/server/Ros2ActionServer.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using namespace std;


class Ros2ActionWaitServer: public Ros2ActionServer {

public:

  /**
   * @brief Construct a new Ros 2 Action Wait Server object
   */
  Ros2ActionWaitServer(rclcpp::Node::SharedPtr node, std::string action)
    :Ros2ActionServer(node, action){    
  }

  /**
   * @brief Destroys the Ros 2 Action Wait Server object
   */
  ~Ros2ActionWaitServer(){}   

private:

  virtual void execute(Ros2ActionContext ros2ActionContext) override {
    
    rclcpp::Rate loop_rate(1);

    int totalLoop =  atoi(ros2ActionContext.getParameters()["time"].c_str());

    for(int i = 0; i <  totalLoop; i++){
        cout<<i<<endl;
        loop_rate.sleep();

        //cancel goal
        if (ros2ActionContext.getGoalHandle()->is_canceling() ) {             
            ros2ActionContext.canceled();            
            return;
        }      

    }  

    // Check if goal is done
    if (rclcpp::ok()) {
        ros2ActionContext.setResult(true);
    } else {
        ros2ActionContext.setResult(false);
    } 

    
  } 

 

  
};

#endif /* INCLUDE_ROS2_ACTION_WAIT_SERVER_H_ */

