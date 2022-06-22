"""
Copyright (C) 2022 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""


def normalize_potentiometer_values(potentiometer_values):
    """
    Transforms all the potentiometer values (usually numbers between 0 and 5) into a numbers between -1 and 1.
    :param potentiometer_values: a list of potentiometer values obtained from the UMIRobot
    :return: a list of the normalized potentiometer values
    """
    return [(potentiometer_value - 2.5) / 5.0 for potentiometer_value in potentiometer_values]
