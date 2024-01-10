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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=2.10,7825.14");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1789.27,7963.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2999.63,7855.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4649.99,7847.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5291.84,7983.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5960.88,7788.03");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=384.76,7658.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=739.88,7520.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2403.79,7629.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3466.05,7597.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3902.48,7612.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4274.15,7702.27");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4966.43,7519.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5556.06,7654.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=214.16,7337.90");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1505.73,7493.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1875.84,7421.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2604.31,7310.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3022.37,7354.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5827.91,7403.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=576.88,7136.28");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1337.37,7185.06");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2161.95,7171.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3657.64,7103.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4009.34,7164.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4462.83,7074.80");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5414.48,7114.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=286.01,6940.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1756.87,6917.74");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2605.79,6886.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3054.29,6935.98");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3455.82,6813.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4875.02,6996.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5887.00,6994.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=304.24,6577.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=835.85,6726.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1205.47,6631.46");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2149.52,6673.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2820.95,6517.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3991.76,6697.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4327.37,6552.07");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5245.29,6589.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5579.99,6746.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5930.31,6528.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=636.43,6362.49");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1662.67,6293.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2216.32,6285.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3675.99,6394.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=379.32,6120.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1296.02,6217.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2613.75,6064.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3255.96,6136.91");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4329.93,6027.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4724.95,6147.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5467.74,6229.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5920.64,6149.55");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=980.64,5935.98");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2247.48,5929.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3790.99,5812.82");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5439.19,5858.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=276.26,5763.92");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1633.98,5639.69");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2721.64,5690.84");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4206.72,5687.51");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4892.12,5724.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5855.53,5661.77");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=344.24,5302.84");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=700.21,5475.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1154.89,5289.56");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2238.17,5333.23");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2804.40,5341.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3276.21,5433.36");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3949.39,5436.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4293.42,5335.50");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4736.04,5354.23");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5359.81,5327.59");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1542.88,5174.16");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3376.03,5049.92");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5883.85,5245.73");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=671.79,4850.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2107.86,4927.35");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3793.92,4965.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5067.92,4912.88");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5426.89,4972.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=263.07,4602.61");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1059.68,4768.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1625.50,4582.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3048.26,4698.04");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3514.81,4630.14");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4289.04,4586.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4686.77,4693.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5074.91,4541.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=439.07,4295.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1922.21,4291.34");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=923.43,4200.88");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1291.00,4283.34");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2367.93,4257.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3183.39,4134.38");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3843.63,4275.42");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4220.59,4172.07");
    }

}

