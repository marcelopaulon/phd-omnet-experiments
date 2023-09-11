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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=23.27,132.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=48.77,41.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=165.06,90.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=171.23,166.05");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=64.97,213.24");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=67.78,311.61");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=134.02,325.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=210.60,283.80");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=140.09,249.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=25.21,364.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=9.27,458.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=44.42,508.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=84.46,592.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=168.23,635.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=107.58,689.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=155.09,745.20");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=228.99,735.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=298.68,685.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=366.10,649.86");
    }

}

