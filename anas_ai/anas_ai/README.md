# Logic breakdown of code
  
## position.py
The purpose of this script is simply to detach the calculation of position, orientation, basically all the data from odometry and occupancygrid to a separate node. This note will publish an array of data contain all the above information back to the topic called `position`.
> `arr.data = [float(grid_x), float(grid_y),roll_trans, pitch_trans, yaw_trans, cur_rot.x, cur_rot.y, self.pose_x,self.pose_y,self.roll,self.pitch,self.yaw,]`
  
The idea is to make the codes of the main node less clustered, also if there is no data send from `position` the main code will not run.

## shooting.py
This script is run on the Rpi of the bot to control the shooting mechanism using GPIO.

## occupancy.py
This script purpose is to help visualize what the bot see. Since we do not want the main code to be affected by extra calculation and task such as plotting and showing image which can potentially create lag in execution of crucial tasks like navigating. We use a script outside to do this.  
This script is not much different from the stock script given in class. It was modified to visualize only the line where the unknown and known regions met. It does not send any information to any topic or interact in anyway with the main code.

## anasai.py (unfinished)
This is the main script to run A*Star algorithm. If position topic is not running, A*Star algorithm will run but do nothing.  
(Still in development, works occasionally)

## state.py
Same usage as `position.py` however this is based on automaticaddison logic, so the code was port over to be able to run the wall follower logic.

## wallfollower.py
Mainly adopted from automaticaddison with adjustment to some parameter to fit our usage.



