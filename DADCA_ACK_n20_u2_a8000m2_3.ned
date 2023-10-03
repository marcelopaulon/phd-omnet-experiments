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

network DADCA_ACK_n20_u2_a8000m2_3
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
        @display("i=misc/sensor;p=621.60,7991.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3831.02,7765.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4285.40,7945.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=383.60,7658.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=935.50,7598.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3270.02,7710.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4963.16,7696.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5753.10,7716.39");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=89.63,7437.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4130.65,7455.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4761.17,7376.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5129.04,7267.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5456.44,7444.82");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5974.90,7382.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=228.27,7073.52");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=651.30,7167.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1271.48,7168.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3619.69,7127.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4178.66,7049.03");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5586.21,7116.45");
    }

}

