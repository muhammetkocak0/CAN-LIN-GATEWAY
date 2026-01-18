"""
DBC Parser - Parse CAN Database files
"""

try:
    import cantools
    CANTOOLS_AVAILABLE = True
except ImportError:
    CANTOOLS_AVAILABLE = False


class DBCParser:
    """Parser for DBC (CAN database) files"""
    
    def __init__(self):
        self.db = None
    
    def parse(self, filename):
        """
        Parse DBC file and return structured data
        
        Args:
            filename: Path to DBC file
            
        Returns:
            dict: Parsed DBC data with messages and signals
        """
        if CANTOOLS_AVAILABLE:
            return self._parse_with_cantools(filename)
        else:
            return self._parse_simple(filename)
    
    def _parse_with_cantools(self, filename):
        """Parse using cantools library"""
        self.db = cantools.database.load_file(filename)
        
        messages = []
        for msg in self.db.messages:
            signals = []
            for sig in msg.signals:
                signals.append({
                    'name': sig.name,
                    'start_bit': sig.start,
                    'length': sig.length,
                    'byte_order': 'little_endian' if sig.byte_order == 'little_endian' else 'big_endian',
                    'scale': sig.scale,
                    'offset': sig.offset,
                    'minimum': sig.minimum,
                    'maximum': sig.maximum,
                    'unit': sig.unit,
                    'comment': sig.comment
                })
            
            messages.append({
                'id': msg.frame_id,
                'name': msg.name,
                'length': msg.length,
                'sender': msg.senders[0] if msg.senders else '',
                'signals': signals,
                'comment': msg.comment
            })
        
        return {
            'version': getattr(self.db, 'version', ''),
            'messages': messages
        }
    
    def _parse_simple(self, filename):
        """Simple parser for when cantools is not available"""
        messages = []
        
        with open(filename, 'r') as f:
            lines = f.readlines()
        
        current_message = None
        
        for line in lines:
            line = line.strip()
            
            # Parse message definition: BO_ <ID> <Name>: <Length> <Sender>
            if line.startswith('BO_ '):
                parts = line.split()
                if len(parts) >= 4:
                    msg_id = int(parts[1])
                    msg_name = parts[2].rstrip(':')
                    msg_len = int(parts[3])
                    
                    current_message = {
                        'id': msg_id,
                        'name': msg_name,
                        'length': msg_len,
                        'signals': []
                    }
                    messages.append(current_message)
            
            # Parse signal definition: SG_ <Name> : <StartBit>|<Length>@<ByteOrder><ValueType> ...
            elif line.startswith('SG_ ') and current_message:
                try:
                    parts = line.split()
                    sig_name = parts[1]
                    
                    # Parse bit position and length
                    bit_info = parts[3].split('|')
                    start_bit = int(bit_info[0])
                    
                    length_info = bit_info[1].split('@')
                    length = int(length_info[0])
                    
                    # Parse byte order (0=big, 1=little)
                    byte_order = 'little_endian' if length_info[1][0] == '1' else 'big_endian'
                    
                    # Try to parse scale and offset
                    scale = 1.0
                    offset = 0.0
                    if '(' in line and ')' in line:
                        scale_offset = line[line.find('(')+1:line.find(')')].split(',')
                        if len(scale_offset) >= 2:
                            scale = float(scale_offset[0])
                            offset = float(scale_offset[1])
                    
                    current_message['signals'].append({
                        'name': sig_name,
                        'start_bit': start_bit,
                        'length': length,
                        'byte_order': byte_order,
                        'scale': scale,
                        'offset': offset
                    })
                
                except (IndexError, ValueError):
                    pass  # Skip malformed signals
        
        return {
            'version': '1.0',
            'messages': messages
        }


if __name__ == '__main__':
    # Test
    parser = DBCParser()
    
    # Create a test DBC file
    test_dbc = """
VERSION ""

NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    VAL_
    BA_

BS_:

BU_: Gateway SCADAS_XS

BO_ 256 BLDC_Status_1: 8 Gateway
 SG_ BLDC_Current : 0|8@1+ (0.15,0) [0|38.1] "A" SCADAS_XS
 SG_ BLDC_RPM : 8|8@1+ (25,0) [0|6300] "rpm" SCADAS_XS
 SG_ BLDC_Temperature : 16|8@1+ (1,-40) [-40|210] "C" SCADAS_XS

BO_ 257 BLDC_Status_2: 8 Gateway
 SG_ BLDC_Voltage : 0|10@1+ (0.1,0) [0|102] "V" SCADAS_XS
"""
    
    with open('test.dbc', 'w') as f:
        f.write(test_dbc)
    
    data = parser.parse('test.dbc')
    
    print("Parsed DBC:")
    for msg in data['messages']:
        print(f"  Message: {msg['name']} (0x{msg['id']:03X})")
        for sig in msg['signals']:
            print(f"    Signal: {sig['name']}, Bit: {sig['start_bit']}, Len: {sig['length']}")