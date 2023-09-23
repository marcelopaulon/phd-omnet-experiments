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

network DADCA_ACK_n100_u16_a8000m2_9
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
        @display("i=misc/sensor;p=986.99,7899.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1404.85,7759.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2090.35,7780.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4265.50,7935.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4836.48,7786.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5598.63,7848.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=543.50,7625.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1772.21,7520.78");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2444.58,7683.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4442.56,7620.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5152.59,7590.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=114.28,7305.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5924.84,7406.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=994.11,7211.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1367.31,7137.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2039.56,7169.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2457.90,7212.79");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4689.74,7166.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5373.39,7047.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5960.53,7011.72");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=454.17,6972.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1759.98,6799.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3060.14,6919.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3527.01,6957.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4048.27,7008.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1042.00,6674.62");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2294.50,6563.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2870.28,6559.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3930.99,6650.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4603.70,6733.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5037.60,6744.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5515.19,6535.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=0.89,6498.09");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=537.03,6498.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1333.60,6463.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1693.15,6377.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3640.27,6309.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4949.40,6390.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5918.76,6475.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=952.84,6182.44");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2318.43,6208.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2728.89,6059.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3159.68,6203.60");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4309.30,6263.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5231.27,6100.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5659.14,6156.96");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=474.50,6011.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1174.50,5787.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1521.71,5971.83");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1897.90,5837.44");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3023.78,5829.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3376.15,5854.20");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3832.05,5816.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4663.66,6009.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5693.91,5779.66");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=47.49,5680.40");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=765.80,5740.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2333.49,5738.93");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5270.24,5717.71");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=843.79,5336.35");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1459.55,5398.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2651.90,5512.49");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3099.02,5348.04");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3551.21,5426.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4159.49,5340.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4846.29,5331.72");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5525.30,5347.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=306.05,5067.89");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1263.52,5084.64");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1855.39,5233.98");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2228.75,5251.03");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2645.20,5054.55");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3461.95,5037.74");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3835.99,5106.04");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1568.37,4878.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4466.27,4791.25");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5183.66,4902.84");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5786.18,4909.95");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=170.76,4706.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=943.04,4750.73");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2059.47,4776.04");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2422.28,4732.10");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3207.06,4780.31");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3829.62,4681.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4830.16,4759.27");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5529.27,4664.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=561.60,4531.98");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1341.51,4429.11");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3441.36,4322.65");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4417.59,4380.00");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4897.44,4368.91");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5897.56,4349.04");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=407.24,4211.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=921.71,4217.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1571.62,4160.10");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2150.28,4157.07");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4066.32,4091.65");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5214.66,4217.70");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=151.31,3835.44");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=522.76,3826.19");
    }

}

