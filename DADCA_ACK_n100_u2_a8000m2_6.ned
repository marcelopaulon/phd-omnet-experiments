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
        @display("i=misc/sensor;p=175.51,7919.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1005.02,7927.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1617.19,7752.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2004.20,7841.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2571.44,7822.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3068.94,7827.21");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3570.26,7892.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4392.31,7989.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5448.91,7970.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=584.86,7567.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2131.98,7512.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3504.74,7530.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4117.96,7610.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4515.50,7594.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4839.14,7730.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5233.67,7689.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5877.19,7718.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=76.24,7335.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1689.29,7342.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2633.28,7456.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3117.85,7334.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5578.34,7466.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=434.86,7160.74");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1051.26,7157.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2617.49,7087.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3818.38,7178.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4532.00,7036.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4907.24,7118.75");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5311.74,7096.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=164.75,6809.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1205.84,6836.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1672.16,6984.27");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2251.83,6918.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3174.90,6915.91");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4029.54,6862.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5862.00,6948.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=692.04,6687.47");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1929.14,6689.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3513.34,6600.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3861.85,6520.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4699.40,6701.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5190.99,6719.34");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5828.42,6594.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=706.36,6301.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1081.76,6460.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1544.53,6392.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2296.20,6278.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2914.37,6312.41");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4323.76,6443.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4867.93,6358.11");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5417.89,6326.30");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=302.07,6228.47");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1158.80,6094.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1999.26,6049.71");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2620.83,6049.83");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3230.20,6101.52");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4074.74,6037.34");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=329.70,5803.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3585.83,6014.47");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4514.38,5981.33");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5227.66,5804.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5891.56,5957.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1474.39,5752.60");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2029.26,5595.99");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2686.66,5689.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3104.09,5731.69");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5678.77,5600.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=78.64,5517.41");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=698.01,5501.88");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1181.81,5410.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4084.60,5408.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4949.55,5336.75");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5289.91,5446.75");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1497.64,5214.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2496.08,5073.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3152.28,5102.18");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3712.01,5271.27");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4538.09,5098.00");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=60.45,4826.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=413.77,4928.47");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1033.51,5003.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1533.24,4795.07");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2040.61,4941.70");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2857.11,4796.75");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3693.43,4874.39");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4976.37,4848.06");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5660.42,5002.78");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=527.44,4595.28");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4354.62,4595.72");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=114.19,4452.47");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1747.84,4388.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2082.42,4499.19");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3082.35,4480.11");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3739.26,4368.15");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4119.19,4329.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5399.29,4408.91");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=648.87,4234.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1044.25,4277.32");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1403.36,4174.14");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2542.52,4214.72");
    }

}

