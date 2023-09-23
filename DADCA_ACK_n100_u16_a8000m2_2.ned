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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=483.01,7808.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=869.18,7819.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1653.71,7866.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2150.69,7925.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3602.15,7812.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4025.23,7793.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4940.85,7755.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5396.79,7871.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2497.70,7529.15");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5953.91,7580.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=457.26,7428.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1319.15,7381.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3069.13,7427.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3712.09,7477.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4051.87,7371.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4417.27,7498.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5484.91,7483.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=779.12,7068.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1879.64,7235.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4435.92,7073.72");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5142.86,7095.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5750.56,7136.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1171.74,6812.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2267.12,6907.60");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2704.60,6994.94");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3131.44,6991.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3579.22,6958.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4066.85,6896.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=492.95,6754.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1547.09,6629.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3308.57,6686.46");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4691.36,6652.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5260.77,6635.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5636.73,6639.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=660.74,6387.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2163.32,6308.26");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2970.44,6387.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3642.68,6481.94");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4421.63,6423.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=265.68,6259.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1126.57,6194.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2503.89,6095.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4995.61,6088.31");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5639.52,6178.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=603.95,5870.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=994.51,5807.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1795.24,5952.50");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2267.32,5804.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2806.40,5908.68");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3460.08,5810.64");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4443.31,5955.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=122.54,5549.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1418.60,5720.80");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1889.44,5543.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4096.69,5736.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5357.07,5599.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5780.94,5737.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=346.56,5280.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1297.08,5307.39");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2318.00,5418.40");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3115.86,5320.94");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3826.26,5393.94");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4771.01,5464.30");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=782.94,5158.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1738.47,5154.29");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2698.90,5263.31");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4160.80,5257.69");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5227.27,5199.04");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5821.64,5117.46");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2438.13,4907.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3203.86,4937.07");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3868.90,4923.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4445.05,4938.47");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4812.95,4948.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5490.16,4862.29");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=557.46,4719.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=997.09,4704.04");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1409.34,4742.65");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2032.52,4600.81");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2674.02,4568.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4257.10,4643.12");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4630.09,4566.56");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5141.42,4660.91");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=299.58,4371.27");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=780.04,4387.64");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1736.17,4348.84");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3839.82,4393.31");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5034.80,4321.55");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5657.26,4440.39");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=23.74,4102.07");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1084.53,4179.25");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3363.70,4269.50");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4408.74,4113.88");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=400.50,3946.74");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1337.63,3861.83");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1843.49,3921.17");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2332.22,4023.48");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2954.75,4035.22");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3563.97,3797.25");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4036.39,3849.20");
    }

}

