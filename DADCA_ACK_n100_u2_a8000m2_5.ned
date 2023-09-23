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
        @display("i=misc/sensor;p=0.65,7896.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=400.89,7869.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2530.50,7899.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3131.18,7900.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4697.33,7988.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5475.50,7989.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1112.39,7647.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1648.04,7676.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2046.15,7708.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3622.53,7508.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4238.85,7724.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4883.67,7689.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5819.27,7518.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=153.45,7411.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=838.48,7328.32");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1417.82,7309.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2561.21,7393.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2932.66,7361.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1157.71,7035.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1883.80,7078.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2296.48,7114.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3448.16,7197.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4128.83,7222.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4509.54,7224.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5041.77,7076.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5476.47,7035.29");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=196.33,6959.21");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2932.43,6867.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3940.62,6905.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=773.37,6733.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1504.51,6653.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3301.54,6701.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4830.67,6606.64");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5927.95,6716.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=321.62,6369.86");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1038.26,6397.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1665.78,6316.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2216.67,6514.93");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3063.91,6397.07");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3876.43,6428.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4349.95,6509.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5394.82,6352.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=779.97,6055.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2680.77,6253.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3309.44,6108.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4660.08,6182.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1201.65,5868.74");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2149.22,5999.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3711.90,5950.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4159.82,5972.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4926.69,5862.67");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=216.26,5682.44");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=656.33,5606.20");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1608.13,5675.79");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2752.90,5743.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3464.99,5677.39");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3965.39,5542.77");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4412.68,5559.14");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5852.66,5706.39");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1176.91,5297.05");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2119.91,5510.22");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2609.61,5359.11");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3189.25,5447.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3657.66,5355.05");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5260.33,5512.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5823.03,5304.06");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=146.33,5239.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=627.74,5091.33");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1586.59,5250.99");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2823.10,5051.51");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3342.17,5058.33");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4913.74,5135.36");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2167.03,4940.12");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4322.42,4989.26");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5297.33,5000.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5967.42,4932.23");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=832.16,4710.74");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1720.68,4778.83");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2467.60,4678.03");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3164.68,4720.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3779.92,4613.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4135.31,4576.64");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4881.03,4674.43");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5681.99,4666.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=414.78,4437.53");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1343.95,4347.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2064.17,4429.23");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4791.18,4319.08");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5214.49,4309.58");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5860.09,4332.55");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=700.82,4190.45");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1782.56,4068.62");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2398.47,4125.06");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2743.60,4235.95");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3209.95,4141.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3863.46,4201.24");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4369.32,4204.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1135.48,3908.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2747.06,3885.26");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4082.03,3922.59");
    }

}

