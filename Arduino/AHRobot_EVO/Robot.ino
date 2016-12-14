// JJROBTOS AHR AIR HOCKEY ROBOT EVO PROJECT

// Each time a new data packet from camera is reveived this function is called
void newDataStrategy()
{
  // predict_status == 0 => No risk
  // predict_status == 1 => Puck is moving to our field directly
  // predict_status == 2 => Puck is moving to our field with a bounce
  // predict_status == 3 => ?

  // Default
  robot_status = 0;   // Going to initial position (defense)

  if (predict_status == 1) // Puck comming?
  {
    if (predict_bounce == 0)  // Direct impact?
    {
      if ((predict_x > (ROBOT_MIN_X + (PUCK_SIZE * 2))) && (predict_x < (ROBOT_MAX_X - (PUCK_SIZE * 2))))
      {
        if (puckSpeedYAverage > -280)
          robot_status = 2;  // defense+attack
        else
          robot_status = 1;  // Puck too fast => only defense
      }
      else
      {
        if (predict_time < 500)
          robot_status = 1; //1  // Defense
        else
          robot_status = 0;
      }
    }
    else // Puck come from a bounce?
    {
      if (puckSpeedYAverage > -160) // Puck is moving fast?
      {
        robot_status = 2;  // Defense+Attack
      }
      else
        robot_status = 1;  // Defense (too fast...)
    }

  }

  // Prediction with side bound
  if (predict_status == 2)
  {
    if (predict_time < 500)
    {
      // Limit movement
      predict_x = constrain(predict_x, ROBOT_CENTER_X - (PUCK_SIZE * 4), ROBOT_CENTER_X + (PUCK_SIZE * 4));
      robot_status = 1;   // only defense mode
    }
    else
      robot_status = 0;
  }

  // If the puck is moving slowly in the robot field we could start an attack
  if ((predict_status == 0) && (puckCoordY < (ROBOT_CENTER_Y - 20)) && (myAbs(puckSpeedY) < 60) && (myAbs(puckSpeedX) < 100))
  {
    robot_status = 3;
  }

}

// This function returns true if the puck is behind the robot and there are posibilities of an auto goal when the robots moves back
boolean checkOwnGoal()
{
  if ((real_position_y > (defense_position + PUCK_SIZE)) and (puckCoordY < real_position_y) and (puckCoordX > (ROBOT_CENTER_X - PUCK_SIZE * 5)) and (puckCoordX < (ROBOT_CENTER_X + PUCK_SIZE * 5)))
  {
    Serial.print("AOG ");
    Serial.print(real_position_x);
    Serial.print(" ");
    Serial.println(real_position_y);
    return true;
  }
  else
    return false;
}

// Robot Moves depends directly on robot status
// robot status:
//   0: Go to defense position
//   1: Defense mode (only move on X axis on the defense line)
//   2: Defense + attach mode
//   3: Attack mode
//   4: ?? REMOVE ??
//   5: Manual mode => User send direct commands to robot
void robotStrategy()
{
  max_speed = user_max_speed;  // default to max robot speed and accel
  max_acceleration = user_max_accel;
  switch (robot_status) {
    case 0:
      // Go to defense position
      com_pos_y = defense_position;
      com_pos_x = ROBOT_CENTER_X;  //center X axis
      max_speed = (user_max_speed / 3) * 2; // Return a bit more slowly...
      if (checkOwnGoal() == false)
        setPosition_straight(com_pos_x, com_pos_y);
      else
        setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
      attack_time = 0;
      break;
    case 1:
      // Defense mode (only move on X axis on the defense line)
      predict_x = constrain(predict_x, (PUCK_SIZE * 3), TABLE_WIDTH - (PUCK_SIZE * 3));  // we leave some space near the borders...
      com_pos_y = defense_position;
      com_pos_x = predict_x;
      setPosition_straight(com_pos_x, com_pos_y);
      attack_time = 0;
      break;
    case 2:
      // Defense+attack
      if (predict_time_attack < 150) // If time is less than 150ms we start the attack HACELO DEPENDIENTE DE LA VELOCIDAD?? NO, solo depende de cuanto tardemos desde defensa a ataque...
      {
        com_pos_y = attack_position + PUCK_SIZE * 4; // we need some override
        com_pos_x = predict_x_attack;
        // We supose that we start at defense position
        //com_pos_x = ROBOT_CENTER_X + (((long)(predict_x_attack - ROBOT_CENTER_X) * (com_pos_y - defense_position)) / (attack_position - defense_position));
        setPosition_straight(com_pos_x, com_pos_y);
      }
      else      // Defense position
      {
        com_pos_y = defense_position;
        com_pos_x = predict_x;  // predict_x_attack;
        setPosition_straight(com_pos_x, com_pos_y);
        attack_time = 0;
      }

      break;
    case 3:
      // ATTACK MODE
      if (attack_time == 0)
      {
        attack_predict_x = predictPuckXPosition(500);
        attack_predict_y = predictPuckYPosition(500);
        if ((attack_predict_x > (PUCK_SIZE * 3)) && (attack_predict_x < (TABLE_WIDTH - (PUCK_SIZE * 3))) && (attack_predict_y > (PUCK_SIZE * 4)) && (attack_predict_y < (ROBOT_CENTER_Y - (PUCK_SIZE * 5))))
        {
          attack_time = millis() + 500;  // Prepare an attack in 500ms
          attack_pos_x = attack_predict_x;  // predict_x
          attack_pos_y = attack_predict_y;  // predict_y
          Serial.print("AM:");
          //Serial.print(attack_time);
          //Serial.print(",");
          Serial.print(attack_pos_x);
          Serial.print(",");
          Serial.println(attack_pos_y);
          //Serial.print(" ");
          // Go to pre-attack position
          com_pos_x = attack_pos_x;
          com_pos_y = attack_pos_y - PUCK_SIZE * 4;
          max_speed = user_max_speed / 2;
          if (checkOwnGoal() == false)
            setPosition_straight(com_pos_x, com_pos_y);
          else
            setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
          attack_status = 1;
        }
        else
        {
          attack_time = 0;  // Continue waiting for the right attack moment...
          attack_status = 0;
          // And go to defense position
          com_pos_y = defense_position;
          com_pos_x = ROBOT_CENTER_X;  //center X axis
          max_speed = (user_max_speed / 3) * 2;
          if (checkOwnGoal() == false)
            setPosition_straight(com_pos_x, com_pos_y);
          else
            setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
        }
      }
      else
      {
        if (attack_status == 1)
        {
          long impact_time = attack_time - millis();
          if (impact_time < 170)  // less than 150ms to start the attack
          {
            // Attack movement
            com_pos_x = predictPuckXPosition(impact_time);
            com_pos_y = predictPuckYPosition(impact_time);
            setPosition_straight(com_pos_x, (com_pos_y + PUCK_SIZE * 2));

            Serial.print("ATTACK:");
            Serial.print(com_pos_x);
            Serial.print(",");
            Serial.println(com_pos_y);

            attack_status = 2; // Attacking
          }
          else  // attack_status=1 but it´s no time to attack yet
          {
            // Go to pre-attack position
            com_pos_x = attack_pos_x;
            com_pos_y = attack_pos_y - PUCK_SIZE * 4;
            max_speed = user_max_speed / 2;
            if (checkOwnGoal() == false)
              setPosition_straight(com_pos_x, com_pos_y);
            else
              setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
          }
        }
        if (attack_status == 2)
        {
          if (millis() > (attack_time + 80)) // Attack move is done? => Reset to defense position
          {
            Serial.print("RESET");
            attack_time = 0;
            robot_status = 0;
            attack_status = 0;
          }
        }
      }
      break;
    case 4:
      // The puck came from a bounce
      // Only defense now (we could improve this in future)
      // Defense mode (only move on X axis on the defense line)
      predict_x = constrain(predict_x, (PUCK_SIZE * 3), TABLE_WIDTH - (PUCK_SIZE * 3));
      com_pos_y = defense_position;
      com_pos_x = predict_x;
      setPosition_straight(com_pos_x, com_pos_y);
      attack_time = 0;
      break;

    case 5:
      // User manual control
      max_speed = user_target_speed;
      // Control acceleration
      max_acceleration = user_target_accel;
      setPosition_straight(user_target_x, user_target_y);
      //Serial.println(max_acceleration);
      break;

    default:
      // Default : go to defense position
      com_pos_y = defense_position;
      com_pos_x = ROBOT_CENTER_X; // center
      if (checkOwnGoal() == false)
        setPosition_straight(com_pos_x, com_pos_y);
      else
        setPosition_straight(real_position_x, real_position_y); // The robot stays on it´s position
      attack_time = 0;
  }
}

// Test sequence to check mechanics, motor drivers...
void testMovements()
{
  if (loop_counter >= 9000) {
    testmode = false;
    return;
  }
  max_speed = user_max_speed;
  if (loop_counter > 8000)
    setPosition_straight(200, 60);
  else if (loop_counter > 6260)
    setPosition_straight(100, 200);
  else if (loop_counter > 6000)
    setPosition_straight(320, 200);
  else if (loop_counter > 5000)
    setPosition_straight(200, 60);
  else if (loop_counter > 3250)
    setPosition_straight(300, 280);
  else if (loop_counter > 3000)
    setPosition_straight(200, 280);
  else if (loop_counter > 2500)
    setPosition_straight(200, 60);
  else if (loop_counter > 1500)
    setPosition_straight(200, 300);
  else
    setPosition_straight(200, 60);
}

