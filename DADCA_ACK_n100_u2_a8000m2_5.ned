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

network DADCA_ACK_n100_u2_a8000m2_5
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
        @display("i=misc/sensor;p=684.52,7917.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1820.97,7941.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3166.32,7944.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3680.30,7934.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4487.32,7995.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5080.83,7819.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=222.72,7567.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=777.14,7565.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1260.66,7672.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1646.49,7591.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2594.92,7746.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2891.88,7513.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3540.02,7606.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4141.04,7708.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5590.78,7668.87");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5969.83,7727.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1362.68,7297.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2350.47,7492.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3928.89,7317.09");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4949.39,7474.38");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5352.55,7386.44");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=264.11,7125.82");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1945.84,7104.80");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3614.78,7107.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4514.64,7119.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5979.21,7250.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=509.81,6865.83");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=993.87,6942.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1502.71,6836.21");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2125.07,6766.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2664.61,6879.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3196.55,6913.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5136.07,6800.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5680.80,6894.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=145.24,6645.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=849.78,6609.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2943.88,6602.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3391.17,6516.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3720.79,6640.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4231.92,6696.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1184.30,6337.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1572.86,6336.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2131.50,6310.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2571.37,6432.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4200.09,6304.67");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4829.19,6374.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5342.28,6423.71");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=702.43,6231.51");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3152.80,6219.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5811.56,6039.93");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=363.58,5866.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=920.10,5931.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1348.87,6011.04");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1932.28,5794.62");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2841.29,5938.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3204.68,5868.67");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3589.93,5918.26");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4503.54,5817.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5153.00,5904.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2231.82,5569.55");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=877.15,5439.91");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1504.40,5297.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2814.55,5481.02");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3505.71,5426.88");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3847.23,5344.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4190.71,5479.81");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4874.74,5332.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5701.86,5479.98");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=240.57,5271.54");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1125.05,5037.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1776.73,5054.45");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2137.11,5189.27");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3206.70,5180.00");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=655.41,4987.16");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2472.37,5027.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3912.64,4902.41");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4419.14,4868.39");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4791.88,4879.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5352.02,4902.36");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5927.07,4935.07");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=93.01,4767.62");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2370.44,4654.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2865.87,4707.28");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=558.80,4474.29");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=914.27,4396.78");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1485.98,4441.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1949.24,4522.41");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2581.28,4369.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3533.01,4502.17");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4089.19,4465.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4730.95,4520.63");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5329.22,4482.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5783.47,4335.73");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=261.91,4172.01");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1769.04,4079.28");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2167.18,4182.87");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3095.25,4213.29");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4447.76,4118.95");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5036.19,4099.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=936.74,4012.65");
    }

}

