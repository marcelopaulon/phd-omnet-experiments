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

network DADCA_ACK_n100_u3_a8000m2_1
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
        @display("i=misc/sensor;p=591.05,7793.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1550.39,7870.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2114.74,7907.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2593.60,7965.24");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3081.83,7952.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4213.34,7891.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4568.42,7832.50");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5591.33,7989.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=156.45,7570.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1201.46,7751.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2543.16,7574.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3676.73,7596.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5218.36,7745.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=959.37,7486.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1578.18,7465.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2195.68,7478.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3274.06,7318.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4291.33,7418.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4720.22,7403.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5824.96,7317.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=582.20,7140.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1350.64,7122.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2047.52,7138.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2658.72,7152.45");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4759.81,7017.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5233.72,7150.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=156.93,6885.32");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1046.84,6848.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1658.50,6882.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3227.58,6914.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3723.03,6904.51");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4154.06,6769.63");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5133.80,6801.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5713.75,6854.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=476.22,6578.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1540.42,6525.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2328.30,6696.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5380.73,6523.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=80.40,6294.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=984.95,6408.36");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2148.71,6355.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2593.47,6454.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3010.91,6469.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3672.82,6387.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4300.27,6419.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=500.81,6150.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1063.66,6038.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2583.21,6066.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3326.13,6255.95");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5644.52,6144.97");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1682.34,5868.48");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2208.47,5786.36");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2989.47,5965.87");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3503.64,5899.32");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4119.65,6005.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4973.95,5984.27");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5339.71,5927.78");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5951.07,5843.20");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=230.03,5707.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=631.53,5719.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1306.12,5715.05");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2671.89,5568.35");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4327.18,5548.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4624.15,5747.47");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5746.83,5544.71");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1287.05,5292.75");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1695.26,5501.00");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3765.98,5519.85");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4979.85,5334.74");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=27.34,5169.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=443.70,5199.17");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2160.63,5170.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3211.43,5259.46");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3600.59,5046.26");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4120.37,5245.92");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5955.55,5149.12");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=857.65,4820.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1568.79,4931.53");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1918.21,4828.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2320.63,4798.63");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2695.09,4935.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4420.60,5027.90");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4861.67,4901.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5637.86,4956.91");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=532.83,4564.59");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1436.28,4604.32");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3345.34,4694.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4147.36,4639.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5227.41,4707.79");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1081.06,4533.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2448.37,4327.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2870.67,4511.15");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3711.17,4514.85");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4009.92,4316.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4536.76,4327.97");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5029.68,4328.17");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5798.87,4522.37");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=161.96,4252.22");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=913.22,4158.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1361.15,4112.67");
    }

}

