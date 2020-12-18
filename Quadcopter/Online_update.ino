#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP
int record_the_number_of_triggers = 0;
unsigned long time_when_it_was_first_triggered = 0;
//<<DEF>> int total_data_bits = 361;//3*3*5*8+1
char Single_character_temporary_storage_slot[] = {'`', '`', '`', '`', '`', '`', '`', '`'};
boolean Data_transfer_permission = false;
int even_sum_check = 0;
boolean The_reading_of_this_bit = false;
char Chest_of_drawers_Online_update_parameters[NUMBER_OF_PARAMETERS][NUMBER_OF_TEXTS_FOR_EACH_PARAMETER] = {{'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}, {'`', '`', '`', '`', '`'}};
int How_many_words_do_you_count_now = 0;
int Which_square_is_it_on_now = 0;
#endif

void Online_update()
{
#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP

    if (record_the_number_of_triggers == 0)
    {
        time_when_it_was_first_triggered = prev_time;
    }
    else if (record_the_number_of_triggers == DATA_TRANSMISSION_START_TRIGGER_AMOUNT - 1)
    {
        if (Absolute_value_function_for_unsigned_long_numbers(time_when_it_was_first_triggered - prev_time) < DATA_TRANSMISSION_START_DELAY)
        {
            Data_transfer_permission = true;
        }
        else
        {
            record_the_number_of_triggers = 0;
        }
    }
    else if (record_the_number_of_triggers == TOTAL_DATA_BITS + DATA_TRANSMISSION_START_TRIGGER_AMOUNT - 1)
    {
        Stop_transmission_routine();
    }
    else
    {
        Unit_transmission_routine();
    }
    record_the_number_of_triggers = (record_the_number_of_triggers == TOTAL_DATA_BITS + DATA_TRANSMISSION_START_TRIGGER_AMOUNT - 1) ? 0 : record_the_number_of_triggers + 1;
#endif
}

unsigned long Absolute_value_function_for_unsigned_long_numbers(unsigned long unsigned_long_in)
{
    return (unsigned_long_in > 0) ? unsigned_long_in : 0 - unsigned_long_in;
}
void Unit_transmission_routine()
{
#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP

    if (Data_transfer_permission)
    {
        The_reading_of_this_bit = (digitalRead(PARAMETER_ADJUSTMENT_DATA)) ? true : false;
        Single_character_temporary_storage_slot[record_the_number_of_triggers - DATA_TRANSMISSION_START_TRIGGER_AMOUNT - How_many_words_do_you_count_now * 8] = The_reading_of_this_bit;
        char tmp_char_get = ((Single_character_temporary_storage_slot[0]) ? 1 : 0) * 1 +
                            ((Single_character_temporary_storage_slot[1]) ? 1 : 0) * 2 +
                            ((Single_character_temporary_storage_slot[2]) ? 1 : 0) * 4 +
                            ((Single_character_temporary_storage_slot[3]) ? 1 : 0) * 8 +
                            ((Single_character_temporary_storage_slot[4]) ? 1 : 0) * 16 +
                            ((Single_character_temporary_storage_slot[5]) ? 1 : 0) * 32 +
                            ((Single_character_temporary_storage_slot[6]) ? 1 : 0) * 64 +
                            ((Single_character_temporary_storage_slot[7]) ? 1 : 0) * 128;
        if ((record_the_number_of_triggers - DATA_TRANSMISSION_START_TRIGGER_AMOUNT + 1) % 8 == 0)
        {

            if ((How_many_words_do_you_count_now + 1) % NUMBER_OF_TEXTS_FOR_EACH_PARAMETER == 0)
            {
                Chest_of_drawers_Online_update_parameters[Which_square_is_it_on_now][How_many_words_do_you_count_now - NUMBER_OF_TEXTS_FOR_EACH_PARAMETER * Which_square_is_it_on_now] = tmp_char_get;
                Which_square_is_it_on_now++;
            }
            else
            {
                Chest_of_drawers_Online_update_parameters[Which_square_is_it_on_now][How_many_words_do_you_count_now - NUMBER_OF_TEXTS_FOR_EACH_PARAMETER * Which_square_is_it_on_now] = tmp_char_get;
            }
            How_many_words_do_you_count_now++;
        }
        else
        {
            Single_character_temporary_storage_slot[record_the_number_of_triggers - DATA_TRANSMISSION_START_TRIGGER_AMOUNT - How_many_words_do_you_count_now * 8] = (digitalRead(PARAMETER_ADJUSTMENT_DATA)) ? true : false;
        }
        even_sum_check += (The_reading_of_this_bit) ? 1 : 0;
    }
#endif
}
void Stop_transmission_routine()
{
#ifdef ONLINE_PARAMETER_ADJUSTMENT_PIN_GROUP

    The_reading_of_this_bit = (digitalRead(PARAMETER_ADJUSTMENT_DATA)) ? true : false;
    even_sum_check += (The_reading_of_this_bit) ? 1 : 0;
    Data_transfer_permission = false;
    if (even_sum_check % 2 == 0)
    {
        double result;
        double array_of_set_tunings[NUMBER_OF_PARAMETERS];
        char tmp_char_arry[NUMBER_OF_TEXTS_FOR_EACH_PARAMETER];
        char *eptr;
        for (int i_strtod = 0; i_strtod < NUMBER_OF_PARAMETERS; i_strtod++)
        {
            tmp_char_arry[0] = Chest_of_drawers_Online_update_parameters[i_strtod][0];
            tmp_char_arry[1] = Chest_of_drawers_Online_update_parameters[i_strtod][1];
            tmp_char_arry[2] = Chest_of_drawers_Online_update_parameters[i_strtod][2];
            tmp_char_arry[3] = Chest_of_drawers_Online_update_parameters[i_strtod][3];
            tmp_char_arry[4] = Chest_of_drawers_Online_update_parameters[i_strtod][4];
            tmp_char_arry[5] = Chest_of_drawers_Online_update_parameters[i_strtod][5];
            tmp_char_arry[6] = Chest_of_drawers_Online_update_parameters[i_strtod][6];
            tmp_char_arry[7] = Chest_of_drawers_Online_update_parameters[i_strtod][7];

            result = strtod(tmp_char_arry, &eptr);
            if (result == 0)
            {

#ifdef SHOW_OUT_OF_RANGE_ERROR_MSG
                Serial.println("online_update_string_to_double_out_of_range ERROR !");
#endif
            }
            else
            {
                array_of_set_tunings[i_strtod] = result;
            }
        }

        roll_controller.SetTunings(array_of_set_tunings[0], array_of_set_tunings[1], array_of_set_tunings[2]);
        pitch_controller.SetTunings(array_of_set_tunings[3], array_of_set_tunings[4], array_of_set_tunings[5]);
        yaw_controller.SetTunings(array_of_set_tunings[6], array_of_set_tunings[7], array_of_set_tunings[8]);
    }
#endif
}
