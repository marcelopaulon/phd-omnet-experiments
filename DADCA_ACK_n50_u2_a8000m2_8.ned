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

network DADCA_ACK_n50_u2_a8000m2_8
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
        @display("i=misc/sensor;p=38.43,7898.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1329.52,7860.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2186.61,7890.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2554.19,7937.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4048.91,7789.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4646.73,7979.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5164.23,7906.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=652.92,7745.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3205.71,7609.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3709.96,7599.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4766.01,7578.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5889.06,7512.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=92.57,7416.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1039.41,7455.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2109.50,7486.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2685.51,7374.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4300.91,7406.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5498.33,7436.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=548.96,7235.29");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1220.61,7133.74");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1710.54,7232.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2384.01,7157.78");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2787.07,7011.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3640.04,7226.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4623.88,7188.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=979.60,6822.11");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3142.04,7009.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3882.54,6937.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5909.29,6991.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=90.11,6759.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1722.77,6595.55");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2196.11,6543.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2668.16,6647.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3138.33,6563.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3570.57,6722.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4467.07,6605.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4852.02,6577.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5376.99,6615.37");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=521.83,6462.87");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1117.81,6423.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3668.18,6310.93");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5101.90,6281.24");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5743.10,6276.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=231.27,6193.94");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1886.23,6184.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2668.69,6139.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3256.42,6079.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4190.11,6083.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4780.13,6032.13");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=442.66,5860.16");
    }

}

