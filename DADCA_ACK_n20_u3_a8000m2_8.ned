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

network DADCA_ACK_n20_u3_a8000m2_8
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
        @display("i=misc/sensor;p=443.46,7832.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1352.61,7882.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2577.48,7756.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3148.11,7965.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3616.05,7827.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4285.46,7902.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=777.98,7602.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1218.21,7526.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3947.86,7608.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4787.73,7678.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5205.00,7735.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=459.76,7276.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1602.06,7382.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3156.57,7418.04");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3524.98,7273.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4320.36,7291.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5475.93,7487.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=952.72,7064.15");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2370.90,7072.19");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2902.17,7152.47");
    }

}

