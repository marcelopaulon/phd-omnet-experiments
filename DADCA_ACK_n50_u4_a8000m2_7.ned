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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=38.53,7798.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=344.32,7997.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3393.06,7820.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3873.11,7934.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4505.43,7957.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4964.92,7778.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5745.62,7924.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=290.50,7519.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=646.52,7710.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1041.33,7752.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1580.36,7721.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2066.47,7729.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4156.33,7670.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5320.82,7712.61");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5988.60,7532.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=792.94,7285.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1145.16,7290.72");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1690.99,7337.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2359.34,7472.94");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2845.46,7461.86");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3478.08,7274.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4687.03,7444.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5678.05,7308.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=194.26,7121.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1860.54,7021.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2392.73,7038.85");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2795.83,7101.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3168.53,7072.77");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4450.86,7171.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5034.28,7204.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1193.06,6874.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4671.69,6833.35");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5470.08,6932.78");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=686.41,6630.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1618.31,6698.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1931.11,6526.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2854.76,6723.85");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3177.77,6571.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3779.33,6670.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4349.81,6624.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5479.04,6560.45");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5827.46,6613.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=205.90,6437.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1411.53,6412.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2419.50,6460.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4888.27,6335.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=508.74,6217.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1017.29,6265.01");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2679.58,6218.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3237.14,6032.35");
    }

}

