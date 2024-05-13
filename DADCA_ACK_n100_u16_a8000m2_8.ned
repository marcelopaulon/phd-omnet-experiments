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

network DADCA_ACK_n100_u16_a8000m2_8
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
        @display("i=misc/sensor;p=609.16,7776.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=934.49,7908.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1575.56,7817.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2502.40,7984.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3017.09,7956.74");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4445.03,7823.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5624.93,7764.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2242.28,7696.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2753.38,7578.31");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3287.76,7603.64");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3641.04,7672.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4006.21,7566.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4928.06,7531.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=824.77,7337.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1168.83,7458.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2078.29,7284.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2453.24,7358.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5167.67,7268.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5632.73,7390.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=347.95,7039.24");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1557.75,7240.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3032.98,7087.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3507.13,7237.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4181.82,7242.01");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4680.44,7161.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1109.80,6928.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1544.54,6869.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2045.81,6803.81");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3820.35,6926.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5393.87,6931.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=919.70,6543.68");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1369.71,6518.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2963.62,6603.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3608.64,6603.85");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4148.29,6691.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4790.96,6592.66");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=465.22,6411.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1933.56,6395.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2338.30,6308.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2718.20,6303.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4210.49,6335.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5806.53,6462.19");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3049.59,6125.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3870.08,6243.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4528.04,6074.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4898.96,6090.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5424.42,6260.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=573.95,5781.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1042.67,5912.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1478.56,5969.98");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1956.58,5871.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2435.37,5848.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3543.02,5933.04");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5268.32,5908.71");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5874.52,5937.97");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=35.95,5731.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3120.55,5688.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4880.86,5706.24");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=313.03,5464.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=889.80,5390.73");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1410.45,5524.49");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1845.68,5399.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2345.33,5321.34");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2689.91,5458.27");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3464.08,5337.46");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3874.48,5315.67");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4232.09,5460.62");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5028.52,5334.57");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5956.04,5332.68");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3063.48,5257.90");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3664.62,5033.86");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5374.53,5270.32");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=283.30,4918.14");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1085.76,5008.37");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2261.17,4955.98");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2647.59,4991.85");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3980.10,4870.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4300.07,5018.42");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4891.12,4901.46");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5491.43,4882.70");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5954.14,4884.50");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=802.24,4663.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1354.05,4654.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1932.00,4573.51");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3119.63,4704.36");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4284.98,4637.83");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5111.71,4615.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=227.89,4367.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=735.94,4307.44");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2434.74,4414.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2912.14,4398.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3579.01,4419.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3993.87,4324.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4656.80,4532.36");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5693.68,4324.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1546.84,4106.81");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5346.11,4168.76");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=259.52,3993.97");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=955.90,4007.10");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1993.48,4026.03");
    }

}

