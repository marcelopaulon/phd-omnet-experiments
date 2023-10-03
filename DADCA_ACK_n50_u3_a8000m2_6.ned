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

network DADCA_ACK_n50_u3_a8000m2_6
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
        @display("i=misc/sensor;p=591.23,7989.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1485.46,7999.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1841.22,7851.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2230.32,7918.72");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3200.81,7986.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3790.55,7882.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5835.89,7981.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1137.99,7740.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2737.02,7708.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3165.60,7606.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4686.31,7670.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5441.97,7704.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5848.27,7608.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=228.64,7431.62");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=575.10,7355.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1256.08,7363.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1590.24,7487.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2113.93,7430.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2637.51,7318.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3712.23,7271.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4168.59,7493.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5152.00,7313.33");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=243.49,7022.66");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1050.38,7054.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2403.02,7049.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3254.69,7225.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4503.45,7248.48");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5702.84,7071.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1616.43,6979.39");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1965.06,6819.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2928.42,6920.38");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3290.20,6791.90");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4067.70,6873.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4589.69,6876.12");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5064.38,6857.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=434.73,6636.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1122.35,6566.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5383.67,6712.87");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=32.14,6300.45");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1473.80,6353.12");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2381.57,6336.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2776.49,6501.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3504.31,6490.36");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4165.14,6408.73");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4545.61,6451.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5988.44,6462.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=715.07,6215.70");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1119.13,6138.50");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1994.10,6020.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3077.21,6082.63");
    }

}

