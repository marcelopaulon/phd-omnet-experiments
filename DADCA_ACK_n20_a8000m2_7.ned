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

network DADCA_ACK_n20_a8000m2_7
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
        @display("i=misc/sensor;p=42.03,7961.28");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1140.10,7811.38");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2620.13,7777.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4427.53,7946.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5541.02,7884.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=630.43,7717.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1615.32,7750.56");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2321.75,7543.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3006.23,7684.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3904.88,7630.21");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4901.06,7677.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5719.24,7547.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=67.23,7424.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=433.23,7422.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=882.68,7329.88");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1407.13,7412.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4533.21,7332.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=295.95,7034.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1157.03,7103.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1754.47,7190.09");
    }

}

