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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=149.97,149.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=84.49,196.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=158.76,12.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=55.83,20.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=9.22,81.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=274.58,41.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=318.49,93.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=338.97,188.70");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=373.89,275.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=420.85,320.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=461.32,379.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=527.62,434.90");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=591.88,376.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=658.25,371.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=587.84,280.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=494.96,287.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=417.05,229.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=474.16,183.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=424.41,112.91");
    }

}

