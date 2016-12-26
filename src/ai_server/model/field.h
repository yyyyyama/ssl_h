#ifndef _AI_SERVER_MODEL_FIELD_H
#define _AI_SERVER_MODEL_FIELD_H
namespace ai_server{
	namespace model{
		class field{
			private:
				int length_;
				int width_;
				int center_radius_;
				int goal_width_;
				int penalty_radius_;
				int penalty_line_length_;
			public:
				int length(){
					return length;
				}
				int width(){
					return width;
				}
				int center_radius(){
					return center_radius;
				}
				int goal_width(){
					return goal_width;
				}
				int penalty_radius(){
					return penalty_radius;
				}
				int penalty_line_length(){
					return penalty_line_length
				}
				void set_length(int length){
					length_=length;	
				}
				void set_width(int width){
					width_=width;
				}
				void set_center_radius(int center_radius){
					center_radius_=center_radius;
				}
				void set_goal_width(int goal_width){
					goal_widthi_=goal_width;
				}
				void penalty_radius(int penalty_radius){
					penalty_radius_=penalty_radius;
				}
				void penalty_line_length(int penalty_line_length){
					penalty_line_length_=penalty_line_length;
				}
				int x_max(){
					return (length/2);
				}
				int x_min(){
					return (-length/2);
				}
				int y_max(){
					return (width/2);
				}
				int y_min(){
					return (-width/2);
				}
		};
	}
}

#endif    //_AI_SERVER_MODEL_FIELD_H
