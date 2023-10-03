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

network DADCA_ACK_n100_u3_a8000m2_7
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
        @display("i=misc/sensor;p=150.62,7785.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1166.08,7946.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2068.83,7778.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5453.90,7901.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5991.46,7872.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=849.65,7738.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1630.62,7653.42");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4051.15,7718.51");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4890.76,7617.25");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=190.21,7268.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=563.92,7347.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1847.80,7291.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2404.82,7261.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3533.13,7480.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4452.39,7424.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5145.14,7315.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5627.14,7424.62");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1128.83,7185.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2922.67,7246.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3550.78,7062.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3871.03,7244.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4696.10,7125.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=108.85,6882.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=700.01,6942.57");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1287.79,6778.84");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1931.45,6907.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4353.27,6987.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5123.67,6927.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5588.56,6954.70");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2371.82,6683.55");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3037.59,6681.86");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3407.84,6571.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4641.36,6566.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5071.60,6538.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5927.71,6725.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=43.85,6308.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=565.14,6431.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=943.78,6303.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1667.06,6271.26");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2010.73,6388.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4184.02,6402.21");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5401.26,6354.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5977.24,6341.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2352.71,6133.65");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2777.47,6041.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3391.39,6102.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3755.47,6237.65");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4625.57,6142.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=670.45,6015.40");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1257.41,5954.15");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1954.67,5834.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5107.01,5997.06");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5568.34,5815.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=253.41,5709.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1009.47,5686.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1394.47,5533.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1758.18,5533.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3331.07,5715.41");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3752.10,5721.80");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4494.34,5681.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5892.61,5648.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=519.36,5415.74");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2566.37,5413.21");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4134.83,5512.59");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4674.42,5326.07");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5100.63,5400.26");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1464.53,5094.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2075.51,5245.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3046.35,5181.54");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3677.67,5215.43");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4370.96,5047.73");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4923.44,5072.01");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=433.34,4998.13");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=968.47,4970.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2518.38,4831.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2953.68,4819.66");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3313.76,4920.49");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3958.21,4986.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5542.36,5026.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1516.15,4567.39");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1857.58,4755.76");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3620.63,4631.99");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4488.84,4626.75");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4843.43,4681.10");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5862.33,4567.17");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=747.06,4473.48");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1140.33,4498.14");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2336.93,4392.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2983.20,4404.76");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4075.22,4419.69");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5289.98,4379.65");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1014.16,4098.57");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1863.87,4120.43");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2618.34,4147.87");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3318.69,4111.22");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4794.47,4197.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1657.18,3804.70");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3659.39,4027.83");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4330.06,3945.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4857.37,3810.37");
    }

}

