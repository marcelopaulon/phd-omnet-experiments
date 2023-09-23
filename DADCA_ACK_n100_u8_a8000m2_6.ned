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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=721.08,7990.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1596.73,7791.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2011.37,7924.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2456.91,7862.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3153.73,7961.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4391.51,7759.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1231.45,7556.41");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3419.85,7553.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3755.71,7672.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=633.08,7397.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1821.51,7461.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2268.12,7281.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2746.78,7408.67");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4014.06,7318.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=291.99,7187.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=916.53,7016.14");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1501.09,7138.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2849.84,7014.43");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3438.35,7079.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5082.46,7014.57");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=46.51,6859.53");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1325.02,6818.38");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1965.83,6803.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2316.01,6927.22");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3927.06,6853.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4370.15,6942.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=576.37,6643.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1628.37,6572.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5451.57,6621.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5916.58,6530.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=221.30,6484.38");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1197.12,6463.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2628.19,6502.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3161.69,6466.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3593.80,6380.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4674.81,6466.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5130.94,6385.80");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=66.22,6166.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=725.88,6124.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1438.22,6032.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1780.43,6192.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2303.28,6181.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2834.68,6021.02");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3237.31,6083.20");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3863.95,6070.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4213.66,6138.69");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4489.32,5877.97");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4847.55,6017.43");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5298.52,6004.35");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5672.13,6007.14");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=125.54,5556.60");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=527.83,5708.72");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=944.26,5619.43");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2123.18,5629.18");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3111.60,5698.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5047.54,5564.07");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5483.20,5704.86");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5833.13,5686.61");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2481.17,5449.38");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3391.18,5447.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3924.00,5524.98");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4200.03,5281.15");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4569.69,5340.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5709.51,5333.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=119.68,5174.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=501.96,5227.19");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1031.92,5107.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1613.12,5171.41");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2847.95,5152.78");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5334.76,5183.28");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1578.35,4787.51");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2108.40,5027.63");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3522.45,5009.87");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3879.76,4974.09");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4754.49,4970.38");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=771.96,4660.29");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1164.72,4658.01");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2722.15,4652.65");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3145.13,4712.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4184.71,4738.53");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5511.23,4756.14");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5903.87,4677.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=74.22,4365.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1859.24,4519.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2275.93,4366.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3307.23,4382.58");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3912.87,4444.43");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4802.83,4448.24");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5187.55,4515.92");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=514.03,4186.49");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1684.98,4117.63");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4512.92,4195.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5667.50,4162.41");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=0.93,3832.51");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1126.75,3863.86");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2981.52,3917.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3387.74,3793.70");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4102.49,3933.33");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5125.86,3898.34");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=566.24,3567.35");
    }

}

