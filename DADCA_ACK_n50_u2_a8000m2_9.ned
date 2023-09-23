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

network DADCA_ACK_n50_u2_a8000m2_9
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
        @display("i=misc/sensor;p=1343.63,7855.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1705.96,7959.78");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2275.65,7794.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3588.32,7930.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4789.80,7987.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5305.92,7882.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5899.68,7961.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1919.17,7671.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3348.81,7656.56");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4300.59,7656.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5702.95,7545.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=807.90,7314.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1413.41,7340.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2147.92,7400.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2753.08,7440.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4534.42,7381.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4912.16,7468.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5332.19,7479.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=417.23,7226.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2475.07,7025.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3479.46,7029.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3824.15,7212.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5883.02,7127.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1385.80,6961.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1796.29,6913.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3072.64,6894.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4061.85,6911.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4802.10,6857.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=209.61,6749.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=675.81,6627.46");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1241.10,6569.78");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2344.24,6608.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3741.88,6599.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4436.53,6672.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5404.69,6701.18");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5937.38,6626.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=854.08,6274.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1571.46,6318.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2060.36,6402.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2895.68,6352.36");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3392.87,6442.77");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3969.70,6318.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4555.57,6330.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4956.75,6473.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=356.04,6115.30");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1283.10,6083.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2508.72,6161.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3512.56,6065.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4935.02,6030.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5239.64,6235.79");
    }

}

