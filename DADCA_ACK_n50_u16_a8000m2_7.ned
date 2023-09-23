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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=4385.52,7981.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4946.92,7834.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5583.84,7777.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=515.64,7693.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=933.27,7519.92");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1503.25,7699.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2197.39,7734.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3666.83,7648.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4251.38,7643.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=406.12,7336.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1984.83,7368.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2437.87,7441.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3460.46,7263.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4572.25,7452.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5287.20,7435.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5645.68,7427.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=982.74,7059.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1470.50,7161.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2334.92,7058.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2916.30,7201.78");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3991.00,7096.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4957.98,7122.83");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=145.13,6988.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=682.87,6801.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1253.77,6768.86");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1703.79,6847.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3243.09,6811.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4749.71,6791.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5195.13,6863.84");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=164.62,6592.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2166.05,6646.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2595.02,6618.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3642.30,6660.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4192.44,6726.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5678.45,6757.50");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1045.66,6459.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3456.66,6360.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4900.36,6407.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5348.92,6445.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=517.79,6260.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=878.33,6060.02");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1947.46,6037.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3131.99,6194.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3877.27,6103.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4406.05,6078.07");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5832.26,6245.42");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=210.45,5928.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1407.62,5868.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3416.82,5954.38");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4757.18,5890.54");
    }

}

