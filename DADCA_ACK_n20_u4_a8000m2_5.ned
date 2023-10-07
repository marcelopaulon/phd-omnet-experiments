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

network DADCA_ACK_n20_u4_a8000m2_5
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
        @display("i=misc/sensor;p=156.38,7815.43");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=914.98,7854.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1985.91,7788.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2500.42,7940.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3689.22,7940.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4234.73,7782.49");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4846.00,7996.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1512.57,7729.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3227.54,7506.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3842.41,7610.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5049.54,7642.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5735.27,7641.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=18.54,7330.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=548.04,7309.12");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1180.00,7409.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1601.60,7317.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2146.32,7448.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2737.15,7312.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4097.78,7277.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4669.56,7359.50");
    }

}

