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

network DADCA_ACK_n20_u16_a8000m2_7
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
        @display("i=misc/sensor;p=2074.05,7788.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3628.36,7900.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4107.99,7874.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5023.28,7965.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5661.45,7796.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=513.95,7610.53");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1109.38,7709.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1704.00,7679.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2603.43,7581.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3067.21,7661.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4598.92,7665.52");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=88.62,7266.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=763.37,7286.72");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1405.76,7357.14");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1924.53,7387.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3549.87,7445.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3999.30,7492.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5049.82,7294.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5528.60,7420.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=542.43,7010.88");
    }

}

