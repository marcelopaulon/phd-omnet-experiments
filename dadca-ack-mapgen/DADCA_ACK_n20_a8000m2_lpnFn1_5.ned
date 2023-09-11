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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=47.89,62.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=51.34,0.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=29.72,152.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=59.45,215.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=152.07,183.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=164.03,122.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=178.02,3.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=113.31,294.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=31.59,299.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=210.44,225.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=65.54,365.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=224.73,132.33");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=247.63,69.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=290.26,132.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=395.60,66.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=388.68,163.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=347.24,215.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=410.14,270.70");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=464.61,337.63");
    }

}

