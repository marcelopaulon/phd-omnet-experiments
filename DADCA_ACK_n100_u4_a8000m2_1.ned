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

network DADCA_ACK_n100_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1031.23,7759.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1634.48,7984.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2028.08,7965.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2455.37,7902.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3000.63,7943.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4350.22,7922.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5169.97,7988.14");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=213.34,7588.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=662.45,7703.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1588.75,7598.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2065.46,7560.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2837.50,7506.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3289.27,7705.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3940.80,7571.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4747.33,7646.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5100.37,7644.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5646.96,7743.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=33.46,7273.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=955.70,7348.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2557.41,7284.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4409.91,7313.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=552.44,7149.92");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2248.01,7057.11");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3458.64,7206.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3927.99,7067.49");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4999.59,7066.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5463.35,7145.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5814.33,7173.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1143.09,6942.55");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1836.61,7000.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2558.49,6863.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2959.36,6792.54");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5855.24,6818.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=174.82,6634.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=602.01,6701.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1152.30,6564.75");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1516.85,6623.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3519.78,6698.48");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4041.77,6553.03");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4472.73,6695.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4920.29,6594.28");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5306.55,6608.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2132.71,6468.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2790.56,6343.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4388.10,6350.81");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=138.13,6190.67");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=660.63,6170.19");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3123.83,6184.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3743.17,6120.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4648.99,6093.75");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4982.02,6243.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5979.28,6241.11");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=790.38,5787.32");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1360.72,5965.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2057.54,5921.86");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2509.66,6017.36");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4028.75,5857.93");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5503.48,5984.39");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=79.64,5724.59");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1779.85,5604.43");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2844.33,5727.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3391.03,5684.04");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4588.58,5715.26");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5719.80,5577.51");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=579.05,5410.72");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1020.93,5472.24");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2212.98,5521.57");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3648.59,5368.42");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4913.21,5516.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=247.22,5071.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1279.55,5070.13");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1719.60,5206.95");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2527.05,5147.38");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3079.87,5145.53");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4157.49,5233.56");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4912.12,5067.47");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5283.72,5148.25");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2022.66,4874.92");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3007.38,4797.75");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3490.96,4798.62");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3909.03,4803.27");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5558.18,4857.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5919.91,4966.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=538.80,4782.60");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1520.53,4613.25");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2689.16,4645.62");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4210.76,4583.14");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4629.12,4719.96");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=869.16,4370.85");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3255.41,4458.83");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3696.38,4487.24");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5050.08,4469.47");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5667.45,4398.35");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2075.39,4193.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2935.42,4226.31");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4303.92,4219.35");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4774.68,4162.44");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=448.81,4023.38");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1260.67,3987.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1606.21,3850.03");
    }

}

