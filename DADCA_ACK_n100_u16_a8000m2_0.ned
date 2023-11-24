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
        @display("i=misc/sensor;p=887.33,7809.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1312.11,7968.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3242.64,7964.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3912.68,7921.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4361.21,7973.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1506.50,7523.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1918.55,7731.48");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2279.91,7608.37");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2617.45,7709.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2972.15,7556.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4563.06,7671.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4991.63,7607.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=404.25,7478.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=857.97,7284.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1208.20,7271.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2528.60,7338.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3519.37,7371.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5767.50,7489.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=275.93,7128.99");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1775.35,7129.11");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2212.85,7128.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2838.55,7164.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3190.01,7109.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4146.28,7114.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4712.12,7225.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5255.49,7136.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5756.13,7069.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=142.98,6772.78");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=631.31,6896.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1381.53,6939.42");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2417.63,6804.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3546.54,6762.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4627.90,6868.01");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=979.49,6581.85");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1492.74,6605.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1838.83,6669.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2867.18,6735.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4143.07,6572.98");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5065.87,6748.62");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5405.06,6558.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5855.61,6659.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=584.54,6362.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2028.98,6331.73");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2396.12,6302.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3327.30,6468.48");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3888.43,6326.84");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4509.04,6482.63");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4851.87,6398.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1604.15,6132.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2878.29,6267.18");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4533.18,6119.60");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5078.71,6101.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5494.64,6212.55");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5852.87,6190.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=341.85,6010.79");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=912.60,5886.92");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1262.91,5890.49");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1912.34,5846.21");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2262.58,5898.45");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3114.43,5941.11");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3732.87,5844.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4181.77,5945.86");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4873.66,5797.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5649.68,5813.80");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=83.59,5606.84");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=596.91,5556.56");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1558.47,5657.77");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2959.98,5620.03");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3405.75,5638.74");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4332.02,5548.05");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1020.63,5385.32");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1937.22,5350.58");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2355.90,5306.54");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3470.88,5283.10");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5272.23,5372.37");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5857.58,5481.36");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=306.24,5203.88");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2854.91,5058.52");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3887.01,5271.22");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4425.94,5062.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4954.30,5173.61");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=645.03,4982.76");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1087.36,4906.14");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1628.29,4960.83");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5525.32,4987.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=294.90,4613.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=783.33,4641.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1323.33,4644.48");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2651.10,4773.51");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3142.25,4767.53");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3945.56,4771.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4352.64,4680.00");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5007.09,4745.91");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5620.11,4559.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1121.12,4335.29");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1838.38,4427.65");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2854.34,4462.52");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3585.64,4475.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4674.75,4418.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5043.07,4305.28");
    }

}

