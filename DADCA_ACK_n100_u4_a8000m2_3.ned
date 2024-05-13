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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=753.47,7831.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2552.12,7975.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2866.03,7757.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3249.85,7796.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3871.60,7856.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4483.67,7990.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=309.82,7632.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1206.71,7703.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1634.07,7681.22");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2130.92,7624.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4229.73,7713.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=32.12,7410.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=890.82,7386.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1435.74,7385.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2497.78,7410.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2916.16,7312.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3575.41,7477.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3924.81,7304.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4716.16,7432.71");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=87.80,7053.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=471.35,7199.50");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1462.95,7028.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1996.00,7248.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2286.04,7015.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4354.03,7242.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5116.73,7034.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=969.41,6830.52");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2831.59,6969.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3301.04,6911.86");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3852.63,6860.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4405.27,6879.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=291.75,6739.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1560.58,6610.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1985.49,6742.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2571.33,6650.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3658.57,6523.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4710.35,6654.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5365.69,6625.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=614.29,6384.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1016.34,6291.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1368.61,6270.15");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1797.72,6281.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2263.17,6446.94");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3034.74,6420.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4339.77,6397.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4726.12,6303.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5274.29,6270.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=255.74,6110.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2373.92,6023.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3038.39,6043.14");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3495.18,6026.58");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5624.08,6089.96");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=798.46,5814.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1858.61,5905.35");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4473.54,5869.65");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4992.08,5913.12");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=433.85,5614.35");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1259.15,5705.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2215.44,5623.14");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2984.17,5557.17");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4231.98,5596.60");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5394.06,5607.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5790.81,5705.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=110.26,5466.28");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1564.85,5408.87");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2643.70,5425.30");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3808.42,5413.23");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4515.98,5337.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4876.24,5367.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5567.94,5298.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=773.07,5251.14");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1577.98,5035.55");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1885.54,5215.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3275.63,5172.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4214.73,5130.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5153.99,5117.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=151.86,5010.66");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=515.72,5002.40");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2185.31,4967.60");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3007.20,4938.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3548.19,4877.15");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4556.43,4977.41");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=531.30,4560.54");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1760.63,4628.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2445.75,4731.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3951.55,4776.34");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5081.86,4760.83");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5686.37,4707.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=54.24,4318.84");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1221.23,4527.92");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2358.38,4390.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2789.70,4410.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3562.94,4518.96");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4476.47,4530.90");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4840.28,4463.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5377.65,4408.67");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1751.15,4064.17");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3153.42,4271.42");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3784.82,4232.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5734.85,4208.17");
    }

}

