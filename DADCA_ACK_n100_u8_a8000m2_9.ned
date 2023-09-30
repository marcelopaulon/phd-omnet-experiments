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

network DADCA_ACK_n100_u8_a8000m2_9
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
        @display("i=misc/sensor;p=407.96,7944.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=896.78,7898.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1485.48,7853.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1963.63,7783.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2593.30,7936.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4843.01,7830.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5700.21,7821.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1148.22,7563.68");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5241.44,7728.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=307.53,7497.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1425.14,7277.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2132.02,7269.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2704.60,7333.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3088.94,7431.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=455.34,7061.45");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=803.11,7208.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4649.01,7066.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5014.47,7229.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5420.79,7217.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5961.28,7241.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1442.86,6904.00");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1867.67,6969.21");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3475.69,6861.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3827.58,6994.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5708.03,6797.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=427.50,6545.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1061.38,6749.67");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2353.23,6679.91");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3320.07,6521.09");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4187.20,6736.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5065.33,6672.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5993.47,6581.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1073.22,6294.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1506.78,6345.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1929.73,6381.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2488.70,6300.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2905.67,6439.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4446.18,6275.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4789.25,6421.38");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=68.34,6138.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3789.42,6266.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5141.99,6163.94");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5506.30,6117.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=352.52,5893.39");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=709.66,5920.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1595.52,5977.92");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2177.82,5867.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2788.50,5828.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3288.32,5902.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4267.96,5973.40");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4937.19,5809.26");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5432.93,5774.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5843.49,5964.97");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1256.96,5657.92");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1664.92,5630.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3648.54,5743.44");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=275.11,5506.56");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=856.73,5482.97");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1852.04,5324.40");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2349.69,5444.85");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3147.10,5312.11");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4554.85,5494.10");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2691.84,5090.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3554.21,5210.08");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3952.37,5241.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4461.04,5039.99");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4909.86,5084.56");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5301.22,5162.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5667.50,5031.87");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=89.18,4868.08");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=614.30,4992.35");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1053.77,4977.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1746.10,4954.94");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2170.60,4984.12");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3062.43,4926.46");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3416.30,4800.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3783.28,4854.40");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1859.77,4593.22");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2277.92,4598.78");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4107.57,4617.27");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5255.62,4536.10");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5846.24,4710.19");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=76.67,4398.99");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=617.70,4323.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1454.49,4497.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2583.05,4365.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2981.58,4481.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4683.61,4474.86");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1060.37,4241.68");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1415.89,4107.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1851.59,4176.39");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3335.68,4160.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3781.34,4244.77");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4294.24,4077.26");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5592.71,4095.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5955.10,4255.44");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=139.63,3963.28");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1178.67,3834.65");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2761.55,3998.81");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4761.84,3858.72");
    }

}

