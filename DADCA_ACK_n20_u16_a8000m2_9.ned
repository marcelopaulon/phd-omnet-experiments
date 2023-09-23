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

network DADCA_ACK_n20_u16_a8000m2_9
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
        @display("i=misc/sensor;p=34.23,7808.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=386.77,7849.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=914.70,7813.82");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3959.26,7808.34");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4345.20,7874.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5290.52,7945.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1313.41,7566.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2561.21,7736.18");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3215.85,7560.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3614.09,7731.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4737.19,7711.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5975.59,7571.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=548.27,7484.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=921.82,7451.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2680.87,7393.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4023.82,7277.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4303.90,7497.98");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4677.96,7355.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5521.00,7311.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=183.62,7152.94");
    }

}

