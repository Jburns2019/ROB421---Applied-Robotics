from PIL import Image, ImageDraw, ImageFont
import time
import random
from mistyPy.Robot import Robot
import base64
import cv2

import sounddevice
from scipy.io.wavfile import write

speak = True
bot_connect = False
player_count = 0
ipAddress = "192.168.0.103"

if speak:
    import whisper
    print('Loading transcription model.')
    model = whisper.load_model("small.en")

class ChessBoard():
    def __init__(self, size=800, filename="Chess.jpg"):
        self.col_names = [chr(col_num+65) for col_num in range(8)]
        self.row_names = [str(row_num+1) for row_num in range(8)]
        self.board_positions = {}
        self.board_grid = []
        for col_name in self.col_names:
            board_cells = []
            for row_name in self.row_names:
                self.board_positions[col_name+row_name] = 0
                board_cells.append(0)
            
            self.board_grid.append(board_cells)

        self.filename = filename

        self.reset(size)

    def save(self, filename="Chess.jpg", display=True):
        if display:
            self.image.resize((2000, 2000)).save(filename, optimize=True)
            # self.image.show()
        else:
            return

    def reset(self, size=800):
        self.size = (size, size)
        #Create image with white background.
        self.image = Image.new('RGB', self.size, color='white')

        #Make image drawable.
        self.draw = ImageDraw.Draw(self.image)

        for x in range(0, self.size[0], int(self.size[0]/8)):
            for y in range(0, self.size[1], int(self.size[1]/8)):
                self.board_positions[chr(int(8*y/self.size[1])+65)+str(int(8*x/self.size[0])+1)] = (y, self.size[0]-x-int(self.size[0]/8), y+int(self.size[1]/8), self.size[0]-x)
                self.board_grid[7-int(8*x/self.size[0])][int(8*y/self.size[1])] = chr(int(8*y/self.size[1])+65)+str(int(8*x/self.size[0])+1)

        for key in self.board_positions:
            if key[0] in ['B', 'D', 'F', 'H'] and not int(key[1])%2 or key[0] in ['A', 'C', 'E', 'G'] and int(key[1])%2:
                self.draw.rectangle(self.board_positions[key], fill="Black")
            
            left_corner = (self.board_positions[key][0], self.board_positions[key][1])
            self.draw.text((left_corner[0]+int(self.size[0]/8)-30, left_corner[1]+int(self.size[0]/8)-25), key, 'red', ImageFont.truetype("arial.ttf", 20))
            
        self.save(self.filename)
    
    def display_matrix(self):
        display_text = ''
        for x in range(len(self.board_grid)):
            for y in range(len(self.board_grid[0])):
                display_text += self.board_grid[x][y] + '\t'

                if y == len(self.board_grid) - 1:
                    display_text += '\n'
        
        return display_text

class Player():
    def __init__(self, board=None, color='White'):
        self.board = board
        if self.board == None:
            self.board = ChessBoard()
        
        self.color = color
        
        self.piece_types = ['Rook', 'Night', 'Bishop', 'King', 'Queen', 'Pawn']
        self.piece_loc_by_name = {
            'Rook-1': 'A1' if color == 'White' else 'A8',
            'Night-1': 'B1' if color == 'White' else 'B8',
            'Bishop-Black': 'C1' if color == 'White' else 'F8',
            'Queen': 'D1' if color == 'White' else 'D8',
            'King': 'E1' if color == 'White' else 'E8',
            'Bishop-White': 'F1' if color == 'White' else 'C8',
            'Night-2': 'G1' if color == 'White' else 'G8',
            'Rook-2': 'H1' if color == 'White' else 'H8',
        }

        for i in range(8):
            self.piece_loc_by_name[f'Pawn-{i+1}'] = f'{chr(i+65)}2' if color == 'White' else f'{chr(i+65)}7'
        
        self.draw_pieces()
        
    def draw_pieces(self, reset=False):
        if reset:
            self.board.reset()
        
        for name in self.piece_loc_by_name:
            tl_corner = self.board.board_positions[self.piece_loc_by_name[name]]
            self.board.draw.text((tl_corner[0]+35, tl_corner[1]+30), name[0], "white" if self.color=="White" else "black", ImageFont.truetype("arial.ttf", 40), stroke_width=1, stroke_fill="black" if self.color=="White" else "white")
        self.board.save(self.board.filename)
    
    def find_cell(self, loc):
        for x in range(len(self.board.board_grid)):
            for y in range(len(self.board.board_grid[0])):
                if self.board.board_grid[x][y] == loc:
                    return [x, y]
        return []
    
    def manhatten_distance(self, start, end):
        return abs(start[0]-end[0]) + abs(start[1]-end[1])

    def path_find(self, matrix=[[1 for i in range(8)] for j in range(8)], valid_moves=[(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)], start=(0, 0), end=(7,7)):
        path = [start]
        while self.manhatten_distance(path[-1], end) > 0:
            valid_dir_distances = {}
            for dir in valid_moves:
                proposed_loc = (path[-1][0] + dir[0], path[-1][1] + dir[1])
                
                if 0 <= proposed_loc[0] < len(matrix) and 0 <= proposed_loc[1] < len(matrix[0]):
                    valid_dir_distances[self.manhatten_distance(proposed_loc, end)] = proposed_loc
            
            lowest_distance = min(list(valid_dir_distances.keys()))
            path.append(valid_dir_distances[lowest_distance])
        
        return path

    def get_movement_options(self, name):
        board = self.board

        piece_type = 'Pawn'
        for piece in self.piece_types:
            if piece in name:
                piece_type = piece

        end_points = []
        paths = []
        for x in range(len(board.board_grid)):
            for y in range(len(board.board_grid[0])):
                if self.is_valid(name, board.board_grid[x][y]):
                    end_points.append(board.board_grid[x][y])
                    if (piece_type == 'Night' or piece_type == 'King') and not board.board_grid[x][y] in list(self.piece_loc_by_name.values()):
                        paths.append([board.board_grid[x][y]])
        
        start_loc = self.piece_loc_by_name[name]
        start_cell = self.find_cell(start_loc)
        if piece_type != 'Night' and piece_type != 'King':
            for end_point in end_points:
                end_cell = self.find_cell(end_point)
                valid_moves = []
                if piece_type == 'Rook':
                    valid_moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                elif piece_type == 'Bishop':
                    valid_moves = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
                elif piece_type == 'Queen':
                    valid_moves = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
                elif piece_type == 'Pawn':
                    valid_moves = [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)]
                
                path = self.path_find(self.board.board_grid, valid_moves, start_cell, end_cell)
                bad_path = False
                for location_index in range(1, len(path)):
                    if not bad_path:
                        path[location_index] = self.board.board_grid[path[location_index][0]][path[location_index][1]]
                    
                    if path[location_index] in list(self.piece_loc_by_name.values()):
                        bad_path = True

                if not bad_path:
                    paths.append(path[1:])

        return paths

    def is_valid(self, name, end_loc):
        piece_type = 'Pawn'
        for piece in self.piece_types:
            if piece in name:
                piece_type = piece
        
        curr_loc = self.piece_loc_by_name[name]
        curr_cell = self.find_cell(curr_loc)
        end_cell = self.find_cell(end_loc)
        if len(end_cell) == 0:
            return False
        
        if piece_type == 'Rook':
            return (curr_loc[0] == end_loc[0] or curr_loc[1] == end_loc[1]) and curr_loc != end_loc
        elif piece_type == 'Night':
            return abs(end_cell[0]-curr_cell[0]) == 2 and abs(end_cell[1]-curr_cell[1])== 1 or abs(end_cell[0]-curr_cell[0]) == 1 and abs(end_cell[1]-curr_cell[1])== 2
        elif piece_type == 'King':
            return abs(end_cell[0]-curr_cell[0]) <= 1 and abs(end_cell[1]-curr_cell[1]) <= 1 and curr_loc != end_loc
        elif piece_type == 'Queen':
            return (curr_loc[0] == end_loc[0] or curr_loc[1] == end_loc[1] or abs(end_cell[0]-curr_cell[0]) == abs(end_cell[1]-curr_cell[1])) and curr_loc != end_loc
        elif piece_type == 'Bishop':
            return abs(end_cell[0]-curr_cell[0]) == abs(end_cell[1]-curr_cell[1]) and curr_loc != end_loc
        elif piece_type == 'Pawn':
            at_start = self.color == 'White' and curr_loc[1] == '2' or self.color == 'Black' and curr_loc[1] == '7'
            end_loc_is_forward = (-2 if at_start else -1) <= end_cell[0]-curr_cell[0] < 0 and self.color == 'White' or (2 if at_start else 1) >= end_cell[0]-curr_cell[0] > 0 and self.color == 'Black'
            return end_loc_is_forward and curr_cell[1]==end_cell[1] or abs(end_cell[1]-curr_cell[1]) == 1 and end_cell[0]-curr_cell[0] == (-1 if self.color == 'White' else 1)

        return False

    def display_valid_spots(self, name):
        board = self.board

        display_text = ''
        for x in range(len(board.board_grid)):
            for y in range(len(board.board_grid[0])):
                display_text += str(self.is_valid(name, board.board_grid[x][y])) + '\t'

                if y == len(board.board_grid) - 1:
                    display_text += '\n'
        
        return display_text

class ChessGame():
    def __init__(self, filename="Chess.jpg", player_count=0, misty:Robot=None):
        self.filename = filename

        self.misty=misty

        self.player_count = player_count
        self.player_1_name = 'White'
        self.player_2_name = 'Black'

        if player_count == 0:
            self.player_1_type = 'ai'
            self.player_2_type = 'ai'
        elif player_count == 1:
            rand_bool = bool(random.randint(0, 1))
            self.player_1_type = 'user' if rand_bool else 'ai'
            self.player_2_type = 'ai' if rand_bool else 'user'
        else:
            self.player_1_type = 'user'
            self.player_2_type = 'user'

        self.reset()

        self.piece_values = {
            'Pawn': 1,
            'Night': 3,
            'Bishop': 3,
            'Rook': 5,
            'Queen': 9
        }

    def get_player_score(self, player: Player):
        value = 0
        
        piece_list = list(player.piece_loc_by_name.keys())
        for piece_name in piece_list:
            for piece_value_name in self.piece_values:
                if piece_value_name in piece_name:
                    value += self.piece_values[piece_value_name]
        
        return value

    def reset(self):
        self.board = ChessBoard(filename=self.filename)
        self.player_1 = Player(self.board, self.player_1_name)
        self.player_2 = Player(self.board, self.player_2_name)

        self.was_a_win = False
        self.stale_mate = False
        self.winner = ''

    def get_random_piece_with_moves(self, this_player: Player):
        random_piece = ''
        all_options = []
        
        if len(list(this_player.piece_loc_by_name.keys())) > 0:
            random_piece = random.choice(list(this_player.piece_loc_by_name.keys()))
            counter = 0
            all_options = this_player.get_movement_options(random_piece)
            while len(all_options) == 0 and not self.stale_mate and not self.was_a_win:
                random_piece = random.choice(list(this_player.piece_loc_by_name.keys()))
                counter += 1

                if counter == 10:
                    self.stale_mate = True
                
                all_options = this_player.get_movement_options(random_piece)
        else:
            self.stale_mate = True
        
        return (random_piece, all_options)

    def handle_move(self, piece: str, path: list, this_player: Player, other_player: Player, this_player_name: str):
        proposed_location = path[-1]
        
        for loc in path:
            if loc in list(other_player.piece_loc_by_name.values()):
                if not 'Pawn' in piece or 'Pawn' in piece and this_player.piece_loc_by_name[piece][0] != path[-1][0]:
                    other_piece = ''
                    for other_piece_name in other_player.piece_loc_by_name:
                        if other_player.piece_loc_by_name[other_piece_name] == loc:
                            other_piece = other_piece_name
                    
                    proposed_location = loc
                    other_player.piece_loc_by_name.pop(other_piece)
                    if other_piece == 'King':
                        self.was_a_win = True
                        self.winner = this_player_name
                    break
                else:
                    return False
            elif 'Pawn' in piece and this_player.piece_loc_by_name[piece][0] != path[-1][0]:
                return False
        
        this_player.piece_loc_by_name[piece] = proposed_location

        spoken_piece_name = piece
        if '-' in spoken_piece_name:
            spoken_piece_name = spoken_piece_name.split('-')[0]
        say_text(f'{spoken_piece_name} to {proposed_location}.', self.misty, this_player_name)

        this_player.draw_pieces(reset=True)
        other_player.draw_pieces()

        return True

    def get_user_move(self):
        correct_format = False
        while not correct_format:
            get_audio(misty=misty)
            spoken_words = get_translation().lower().replace('.', '')

            correct_format = 'move ' in spoken_words and ' to ' in spoken_words and len(spoken_words.split(' ')) == 4 or 'forfeit' in spoken_words
            if spoken_words != 'thank you':
                print(spoken_words)

        piece, end_loc = (None, None)
        if not 'forfeit' in spoken_words:
            piece = spoken_words.split(' ')[1]
            piece = piece[0].upper() + piece[1:]
            if piece == 'Knight':
                piece = 'Night'
            if piece[0] == 'P':
                piece = 'Pawn'
            if piece[0] == 'Q':
                piece = 'Queen'
            if piece == 'Castle':
                piece = 'Rook'
            end_loc = spoken_words.split(' ')[3].upper()

        return (piece, end_loc)

    def auto_make_move(self, this_player: Player, other_player: Player, this_player_name='White'):
        random_piece, all_options = self.get_random_piece_with_moves(this_player)
        
        if not self.stale_mate and not self.was_a_win:
            random_path = random.choice(all_options)

            if not self.handle_move(random_piece, random_path, this_player, other_player, this_player_name):
                self.auto_make_move(this_player, other_player, this_player_name)

            # time.sleep(.05)

    def user_make_move(self, this_player: Player, other_player: Player, this_player_name: str):
        performed_a_move = False
        while not performed_a_move:
            piece, end_loc = self.get_user_move()
            if piece == None or end_loc == None:
                if self.get_player_score(this_player) > self.get_player_score(other_player):
                    self.was_a_win = True
                else:
                    self.stale_mate = True
                
                performed_a_move = True
            else:
                pot_pieces = []
                for piece_name in this_player.piece_loc_by_name:
                    if piece in piece_name:
                        pot_pieces.append(piece_name)
                
                for piece in pot_pieces:
                    path_options = this_player.get_movement_options(piece)
                    for path in path_options:
                        if end_loc == path[-1] and self.handle_move(piece, path, this_player, other_player, this_player_name):
                            performed_a_move = True
                            break
                    if performed_a_move:
                        break
            
                if not performed_a_move:
                    say_text(f'{piece} could not be moved to {end_loc}. Please give a valid piece and move.', self.misty)
        
        return True

    def play_turn(self, name):
        if name == 'White':
            if self.player_1_type == 'ai':
                self.auto_make_move(self.player_1, self.player_2, 'White')
            else:
                self.user_make_move(self.player_1, self.player_2, 'White')
        elif name == 'Black':
            if self.player_2_type == 'ai':
                self.auto_make_move(self.player_2, self.player_1, 'Black')
            else:
                self.user_make_move(self.player_2, self.player_1, 'Black')
    
    def display_according_to_status(self, misty: Robot):
        if misty != None and self.player_count == 1:
            bot_player = self.player_1 if self.player_1_type == 'ai' else self.player_2
            user_player = self.player_2 if self.player_1_type == 'ai' else self.player_1
            
            score_difference = self.get_player_score(bot_player) - self.get_player_score(user_player)
            if 10 < score_difference:
                misty.DisplayImage("e_Aggressiveness.jpg")
            elif 5 < score_difference <= 10:
                misty.DisplayImage("e_ExcstacyStarryEyed.jpg")
            elif 0 < score_difference <= 5:
                misty.DisplayImage("e_Joy2")
            elif score_difference == 0:
                misty.DisplayImage("e_Admiration.jpg")
            elif -5 <= score_difference < 0:
                misty.DisplayImage("e_Anger.jpg")
            elif -10 <= score_difference < -5:
                misty.DisplayImage("e_Rage3.jpg")
            elif score_difference < -10:
                misty.DisplayImage("e_Rage4.jpg")
    
    def talk_according_to_status(self, misty:Robot):
        if misty != None and self.player_count == 1:
            bot_player = self.player_1 if self.player_1_type == 'ai' else self.player_2
            user_player = self.player_2 if self.player_1_type == 'ai' else self.player_1
            
            score_difference = self.get_player_score(bot_player) - self.get_player_score(user_player)
            if 10 < score_difference:
                texts = ["Imagine being as bad as you. Couldn't be me.", "Why do you even bother playing chess?", "Of course I'm better. I am a robot."]
                rand_message = random.choice(texts)
                misty.Speak(rand_message)
            elif 0 < score_difference <= 10:
                texts = ["You can do better.", "Why don't you give me a challenge."]
                rand_message = random.choice(texts)
                misty.Speak(rand_message)
            elif score_difference == 0:
                texts = ["Don't worry about a mistake, just make a move.", "Do you always take so long with your decisions?"]
                rand_message = random.choice(texts)
                misty.Speak(rand_message)
            elif -10 <= score_difference < 0:
                texts = ["Why are you being like this?", "Stop making such good moves."]
                rand_message = random.choice(texts)
                misty.Speak(rand_message)
            elif score_difference < -10:
                texts = ["This game isn't fun anymore.", "I don't want to play anymore.", "Can you let me get some pieces."]
                rand_message = random.choice(texts)
                misty.Speak(rand_message)
                if rand_message == "I don't want to play anymore.":
                    self.stale_mate = True
    
    def act_according_to_status(self, misty:Robot):
        if misty != None and self.player_count == 1:
            bot_player = self.player_1 if self.player_1_type == 'ai' else self.player_2
            user_player = self.player_2 if self.player_1_type == 'ai' else self.player_1
            
            score_difference = self.get_player_score(bot_player) - self.get_player_score(user_player)
            if 10 < score_difference:
                misty.MoveArms(-90, -90, 100, 100)
            elif 0 < score_difference <= 10:
                misty.MoveHead(0, 20, -10, 100)
                time.sleep(1)
            elif score_difference == 0:
                misty.MoveHead(0, 0, 0, 100)
                misty.MoveArms(90, 90, 100, 100)
            elif -10 <= score_difference < 0:
                misty.MoveHead(0, 20, 45, 100)
                time.sleep(1)
            elif score_difference < -10:
                misty.MoveHead(0, 0, -90, 100)
                misty.MoveArms(-90, -90, 100, 100)
                time.sleep(1)
                misty.MoveHead(0, 0, 90, 100)
                misty.MoveArms(90, 90, 100, 100)
                time.sleep(1)
                misty.MoveHead(0, 0, 25, 100)
                misty.MoveArms(0, 0, 100, 100)

import pyttsx3
def say_text(text: str, misty: Robot=None, voice_actor=None):
    if misty==None and speak:
        engine = pyttsx3.init()

        if voice_actor == 'Black':
            voices = engine.getProperty('voices')
            engine.setProperty('voice', voices[1].id)
        engine.say(text)
        engine.runAndWait()
    elif misty != None:
        if voice_actor == 'Black':
            text = 'As black, ' + text
        else:
            text = 'As white, ' + text

        misty.Speak(text)
        time.sleep(2.5/25*len(text))
    elif voice_actor == None:
        print(text)

def get_audio(file_name='output.wav', f_s=50000, seconds=3, misty:Robot=None):
    my_recording = sounddevice.rec(int(seconds*f_s), f_s, 2)

    if misty != None:
        misty.ChangeLED(0, 0, 255)
    sounddevice.wait()
    write(file_name, f_s, my_recording)

    if misty != None:
        misty.ChangeLED(0, 0, 0)

    return my_recording

def get_translation(file_name='output.wav'):
    result = model.transcribe(file_name, fp16=False)
    
    return result["text"].strip()

def convert_image_to_base64(filename: str):
    base64_string = ''
    with open(filename, "rb") as image2string: 
        base64_string = base64.b64encode(image2string.read())
    
    return base64_string

def individual_assignment(misty: Robot):
    chess_game = ChessGame(f'Chess Game.jpg', player_count, misty)
    
    cv2.namedWindow("Chess Game", 0)
    img=cv2.imread("Chess Game.jpg")
    cv2.imshow("Chess Game", img)
    cv2.waitKey(1)

    game_started = False
    game_ended = False
    while chess_game.player_count == 0 or chess_game.player_count > 0 and not game_ended:
        if not game_started and chess_game.player_count == 1:
            say_text('You are playing white.' if chess_game.player_1_type == 'user' else 'You are playing black.', misty)
        elif not game_started and chess_game.player_count == 0 and speak:
            say_text('White has a male voice and Black has a female voice', misty)
        elif not game_started and chess_game.player_count == 2:
            say_text('The game has begun. White, make your move.', misty)
        
        if not game_started:
            game_started = True
            game_ended = False
        
            if misty != None:
                misty.DisplayImage("e_DefaultContent.jpg")
        
        chess_game.talk_according_to_status(misty)
        for player_name in ['White', 'Black']:
            chess_game.display_according_to_status(misty)
            chess_game.act_according_to_status(misty)

            chess_game.play_turn(player_name)

            img=cv2.imread("Chess Game.jpg")
            cv2.imshow("Chess Game", img)
            cv2.waitKey(1)

            if chess_game.was_a_win:
                say_text(f'The chess game was won by {chess_game.winner}. White had a score of {chess_game.get_player_score(chess_game.player_1)}. Black had a score of {chess_game.get_player_score(chess_game.player_2)}.', misty)
            elif chess_game.stale_mate:
                say_text(f'The chess game was won by neither player. White had a score of {chess_game.get_player_score(chess_game.player_1)}. Black had a score of {chess_game.get_player_score(chess_game.player_2)}.', misty)
            
            if chess_game.was_a_win or chess_game.stale_mate:
                chess_game.display_according_to_status(misty)

                if chess_game.player_count == 0:
                    chess_game.reset()
                    game_started = False
                game_ended = True
                break
    
    if misty != None:
        misty.DisplayImage("e_DefaultContent.jpg")

if __name__ == '__main__':    
    misty = None
    if bot_connect:
        try:
            say_text('Attempting to connect.', misty)
            misty = Robot(ipAddress)
            say_text('I connected!', misty)
        except:
            say_text(f"Connection to {ipAddress} failed.", misty)
    
    individual_assignment(misty)