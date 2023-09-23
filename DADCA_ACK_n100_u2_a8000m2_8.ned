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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=770.39,7882.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2440.87,7936.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3283.60,7999.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3734.81,7847.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4212.69,7883.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=208.49,7604.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1053.83,7664.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1416.47,7707.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2095.72,7601.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2898.87,7715.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4725.06,7741.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5130.55,7742.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=978.95,7291.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2628.75,7319.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4093.06,7407.36");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5134.55,7287.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5466.25,7484.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1449.41,7034.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2068.51,7072.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2927.86,7127.43");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3380.41,7241.74");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4532.61,7249.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5637.15,7061.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=21.88,6785.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=377.15,6935.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=911.52,6771.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2691.22,6793.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3282.40,6799.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3931.43,6982.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4904.81,6998.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5964.54,6826.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2091.44,6582.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4428.21,6608.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5147.90,6626.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=124.06,6401.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=724.54,6453.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1520.61,6492.64");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2348.49,6291.76");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2929.16,6364.65");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3629.43,6499.46");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5497.57,6506.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1151.81,6194.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2598.87,6027.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3270.64,6118.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3905.02,6126.95");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4271.91,6030.66");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4584.07,6235.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5942.53,6212.63");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=153.25,5988.53");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=772.09,6013.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1551.73,5987.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1961.71,5924.51");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2901.41,5844.63");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3693.54,5818.31");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4865.80,6018.67");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5362.55,5940.09");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1078.85,5721.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3465.60,5546.90");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4046.79,5718.29");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4410.88,5709.35");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4855.51,5540.21");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5766.86,5725.46");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=553.72,5428.68");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=911.14,5280.11");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1598.22,5301.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2430.02,5518.46");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3028.48,5513.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3772.07,5320.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4224.76,5366.07");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5321.30,5481.38");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2669.57,5142.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3130.90,5087.47");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5647.01,5072.36");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=31.15,4941.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1115.11,4940.01");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2035.61,4967.80");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2433.66,4862.83");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3513.28,4859.81");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4374.39,5017.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5027.01,5022.29");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=315.76,4581.21");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=684.40,4762.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1468.29,4721.25");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3037.83,4685.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3880.34,4680.03");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4326.78,4636.32");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4757.16,4703.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5415.27,4543.91");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=574.05,4305.27");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1170.74,4434.70");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1996.05,4398.61");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3104.62,4332.37");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5080.07,4363.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5930.51,4510.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1513.46,4146.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2503.64,4239.46");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3886.71,4098.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4514.83,4217.48");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5802.99,4098.90");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=487.18,3893.40");
    }

}

