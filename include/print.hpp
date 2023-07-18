#pragma once

enum tags
{
    empty = 0,
    wait,
    movement,
    limits,
    calc,
    keyboard
};
std::string tagText[6] = {"empty", "wait", "movement", "limits", "calc", "keyboard"};
bool excludeTag[6] = {false,
                      true,
                      false,
                      false,
                      false,
                      false};

void print(std::string output, tags tag = tags::empty)
{

    if (excludeTag[tag])
        return;

    std::cout << tagText[tag] + ": " << output << std::endl;
    return;
}