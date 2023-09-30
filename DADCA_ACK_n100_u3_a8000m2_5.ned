package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u3_a8000m2_5
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=171.06,7840.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=706.34,7917.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2709.32,7840.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3867.86,7760.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4794.13,7792.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5418.01,7965.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=592.91,7584.08");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1051.46,7702.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1540.07,7552.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2255.92,7707.66");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3175.70,7635.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3545.74,7562.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4234.86,7644.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5751.60,7707.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1932.93,7503.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2621.78,7395.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3905.41,7370.89");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4552.64,7437.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5053.27,7436.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=249.12,7019.94");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=801.91,7200.08");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1155.98,7253.79");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1703.18,7207.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2194.82,7240.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3256.25,7205.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4247.28,7010.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=906.87,6824.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1408.59,6992.17");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2011.30,6777.83");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2757.97,6930.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3719.82,6945.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4779.10,6902.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5444.75,6774.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5825.15,6937.10");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=80.06,6619.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=525.21,6668.94");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3139.96,6634.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3722.63,6564.42");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4482.45,6560.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=739.01,6330.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1111.71,6464.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2480.29,6361.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4137.17,6378.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5206.99,6393.67");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5931.75,6427.43");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=477.21,6067.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1026.45,6089.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1696.38,6228.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3452.00,6165.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4130.53,6020.23");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5628.36,6129.64");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=751.87,5840.00");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1635.99,5802.89");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2029.30,5927.04");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3643.88,5837.30");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4694.45,5824.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5984.95,5828.62");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=280.63,5563.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1262.90,5684.36");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2562.86,5637.97");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3040.63,5528.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5075.97,5736.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5529.65,5648.67");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=691.97,5410.07");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1607.55,5434.64");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3713.64,5487.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4201.80,5488.22");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5423.12,5307.40");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=243.13,5053.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1019.69,5251.65");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1627.58,5052.51");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1954.01,5277.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2245.92,5073.72");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3440.08,5085.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4117.32,5104.99");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4787.74,5267.42");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5121.82,5083.48");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5838.39,5174.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2768.16,4990.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3797.24,4840.77");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4549.39,4856.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=882.79,4599.20");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1645.14,4671.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2055.99,4708.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3410.85,4651.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4884.55,4612.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5623.12,4607.33");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=314.06,4364.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1307.57,4390.46");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2787.56,4522.26");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3732.61,4293.41");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4293.59,4297.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=16.04,4074.41");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1969.53,4193.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2375.31,4112.46");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3125.53,4182.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4974.20,4063.94");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5500.78,4053.93");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5989.02,4206.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=494.44,3907.97");
    }

}

