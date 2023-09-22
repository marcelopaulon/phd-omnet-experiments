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

network DADCA_ACK_n100_a8000m2_3
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
        @display("i=misc/sensor;p=1150.89,7746.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1512.89,7976.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2035.18,7833.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3635.82,7854.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4314.48,7978.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5185.58,7768.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5737.06,7984.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=630.13,7688.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2470.70,7566.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3242.36,7572.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4598.27,7613.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1580.97,7282.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1970.33,7391.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2872.34,7451.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4146.21,7333.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5060.79,7397.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5536.57,7363.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5951.58,7272.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=370.53,7050.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1108.60,7125.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2345.24,6982.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2728.84,7102.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3322.40,6974.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5153.13,7042.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=38.06,6838.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=879.23,6815.13");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1638.31,6905.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3848.99,6937.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4553.26,6712.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5698.25,6837.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1286.49,6693.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2038.61,6612.16");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2469.50,6532.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4180.60,6694.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4952.63,6481.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5319.94,6702.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=51.07,6428.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=541.95,6357.28");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1222.90,6270.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3058.83,6327.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3567.23,6386.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5326.86,6198.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5692.38,6365.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=391.60,5945.78");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=967.47,5983.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1565.51,6088.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2252.40,5960.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2844.83,5995.95");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3227.44,5929.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4203.51,6050.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4784.66,6148.32");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5896.74,6057.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3627.01,5740.63");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4101.72,5671.25");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4731.07,5774.15");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5646.38,5758.09");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=46.52,5474.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1484.10,5650.68");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1788.16,5436.01");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2568.36,5670.79");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3008.67,5631.25");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5976.40,5465.70");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=738.52,5394.37");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1165.91,5155.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2352.22,5334.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3417.36,5261.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4371.67,5342.94");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5244.18,5368.30");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=484.80,4941.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1653.40,4953.62");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2173.35,4982.04");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2699.22,5147.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3997.47,4961.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5965.48,4924.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1315.56,4694.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2869.25,4790.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4549.56,4658.17");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4914.91,4737.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5574.13,4843.15");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=293.73,4477.19");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=947.32,4501.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1831.47,4507.15");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2316.97,4416.48");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3398.92,4548.06");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3738.72,4396.78");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5243.07,4410.83");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1297.95,4244.32");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2773.80,4168.54");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4296.05,4369.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4770.13,4348.72");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5844.17,4227.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=866.46,4052.15");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1999.17,4078.96");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3158.91,4060.72");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4198.58,4016.04");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5240.60,4039.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=190.83,3805.36");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1732.32,3713.56");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2364.57,3795.48");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3499.34,3721.94");
    }

}

