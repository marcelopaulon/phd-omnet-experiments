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

network DADCA_ACK_n100_u2_a8000m2_6
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
        @display("i=misc/sensor;p=323.33,7978.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=959.56,7979.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1572.63,7836.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2079.29,7815.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2917.28,7915.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3535.64,7768.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4274.90,7939.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=745.48,7677.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2530.80,7681.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4602.72,7592.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5101.54,7548.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=393.36,7275.63");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1668.41,7415.24");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2023.87,7321.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2929.97,7270.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3281.92,7350.02");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3907.07,7461.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4364.76,7320.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1042.51,7238.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3798.21,7090.28");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4593.94,7041.42");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=165.90,6775.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=564.05,6852.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1242.80,6812.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2785.32,6856.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3195.12,6826.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4144.19,6801.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5151.31,6968.47");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1732.29,6558.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2307.11,6735.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3806.14,6624.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4843.45,6666.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=432.02,6499.72");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=850.86,6462.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2103.82,6408.70");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2672.69,6404.09");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3452.58,6487.66");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4268.33,6439.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5043.71,6342.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=301.84,6083.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=807.43,6060.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1236.00,6207.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1654.21,6132.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2433.43,6059.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2957.79,6116.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3586.93,6057.79");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3891.98,6231.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4582.00,6132.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5405.84,6123.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5833.09,6075.66");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=482.91,5773.95");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4309.81,5804.45");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=33.25,5737.58");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=826.35,5665.21");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1593.53,5668.07");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2174.07,5686.34");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2794.53,5725.86");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3141.57,5666.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3720.08,5560.76");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4598.20,5545.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5067.53,5724.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5511.61,5612.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=264.67,5427.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1281.96,5365.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2123.92,5318.54");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2533.59,5488.32");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5239.88,5337.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5974.73,5487.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=90.26,5081.62");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=648.60,5232.27");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1519.87,5062.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3092.87,5255.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3550.56,5170.81");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4155.84,5250.45");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=899.96,4986.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2695.72,5003.20");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3267.51,4827.88");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3813.44,4938.56");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4273.93,4852.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4767.99,4960.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5540.47,4995.16");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5975.34,4927.98");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=429.46,4548.98");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1321.62,4608.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1908.74,4722.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2434.86,4714.55");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5218.75,4645.50");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1074.22,4348.63");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2999.11,4396.90");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3518.84,4410.70");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3884.27,4532.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4291.52,4475.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4883.53,4330.82");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5831.33,4407.16");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=642.95,4091.19");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1527.37,4119.15");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1908.03,4257.22");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2447.55,4162.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3936.78,4105.02");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5347.86,4132.27");
    }

}

