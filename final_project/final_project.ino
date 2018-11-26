void setup() {
  // put your setup code here, to run once:

  // current_state = PATH_PLANNING
  // goal_state = person_to_rescue

}




void loop() {
  // put your main code here, to run repeatedly:

  switch (current_state){
    case PATH_PLANNING:

    /*
     * 
     * Figure out where next person is
     * figure out quickest path from current to goal (goal being a person) with Djikstra
     * if(next_person.floor() != sparki.floor()){
     *  
     *   current_state = WRONG_FLOOR
     * 
     * }else{
     *   
     *   current_state = PATH_FOLLOWING
     *   
     * }
     */

    break;

    case PATH_FOLLOWING:

      /*
       * 
       * Go from (0,0) to person
       * if(unknown object found){
       * 
       *  Determine whether the object is candle or object
       *  Mark candle on the map
       *  Do we put any unknown objects on map?
       *  current_state = PATH_PLANNING
       *  
       * }
       * 
       */

    break;

    case WRONG_FLOOR:

      /*
       * 
       * Figure out shortest path to elevator
       * Would this just be Djikstra's with a goal of elevator?
       * We might not need a state as well as IN_ELEVATOR state
       * 
       */

    break;

    case IN_ELEVATOR:

      /*
       * 
       * Use bluetooh remote here
       * Wait(5000) for map change-out
       * potentially also just wait until remote feedback
       * 
       */

    break;

    case FOUND_OBJECT:

      /*
       *
       *if(found_object != candle){
       *
       *
       *  Go to object
       *  pickup object
       *  current_state = PATH_PLANNING;
       *       
       *}else{
       *
       * update map with candle info
       * current_state = PATH_PLANNING;
       *
       *}
       *
       *
      */
      
    break;

    case CARRY_PERSON:

    /*
     * 
     * We've already determined the object in front of us is a person
     * Go straight to it
     * Close arms around it
     * Set goal to outside/front door/however we want to do this
     * current_state = PATH_PLANNING
     * 
     */
    
  }

}
