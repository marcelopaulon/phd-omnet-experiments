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

network DADCA_ACK_n100_a8000m2_1
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
        @display("i=misc/sensor;p=715.97,7832.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1112.83,7845.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2184.96,7777.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3496.86,7962.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3912.06,7873.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5110.43,7820.58");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=254.88,7624.51");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1569.54,7713.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2701.79,7710.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3234.25,7536.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4583.23,7676.80");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=818.89,7415.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1389.37,7379.34");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1896.80,7257.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3709.85,7387.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4247.81,7378.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5052.07,7280.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5532.49,7399.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=18.42,7022.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2345.36,7156.42");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2905.41,7098.46");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4266.56,7025.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4706.34,7170.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5702.84,7010.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2020.33,6855.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3243.60,6932.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3818.94,6844.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5226.89,6899.27");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=893.54,6752.73");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1459.23,6722.20");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2873.47,6549.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4044.47,6545.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=34.70,6458.65");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=512.78,6347.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1482.93,6279.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2305.14,6469.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3523.12,6426.54");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4660.99,6483.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5585.39,6438.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=125.67,6108.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1013.22,6204.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1852.66,6168.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2339.72,6051.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2982.90,6135.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4208.14,6210.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=865.54,5809.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1623.40,5773.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2703.97,5831.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3261.40,5893.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3874.50,5970.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5229.51,5917.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5661.52,5994.61");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=489.30,5754.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1310.69,5603.92");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1994.49,5716.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4834.48,5754.60");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5826.20,5662.45");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=30.32,5446.34");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=615.25,5327.13");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1005.72,5411.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2277.53,5482.13");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3102.31,5484.82");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3586.29,5493.98");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4057.42,5450.09");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4547.32,5334.89");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5262.57,5474.07");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=32.58,5074.87");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1352.24,5164.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1975.11,5226.05");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2574.43,5161.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4966.49,5124.01");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5915.31,5095.15");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=515.00,4921.77");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1786.06,4914.19");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3018.15,5006.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3584.17,4941.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4164.40,4853.83");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4626.49,4834.32");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=97.69,4675.05");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=961.22,4736.17");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1529.76,4675.22");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2328.66,4567.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5309.08,4761.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5895.11,4564.54");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1.48,4334.59");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=457.14,4324.80");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2736.25,4504.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3337.31,4437.37");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4296.11,4291.44");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5165.12,4441.83");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1050.73,4110.96");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1787.96,4116.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2378.34,4173.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3050.73,4078.87");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3513.68,4069.38");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3866.04,4075.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4650.40,4239.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=395.43,3884.66");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1474.95,3874.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2558.40,3846.25");
    }

}

