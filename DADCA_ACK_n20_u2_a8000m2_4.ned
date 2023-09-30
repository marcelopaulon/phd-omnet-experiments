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

network DADCA_ACK_n20_u2_a8000m2_4
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
        @display("i=misc/sensor;p=364.58,7982.22");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2215.31,7866.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2858.43,7875.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3495.47,7782.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4054.30,7965.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4430.25,7862.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5594.49,7937.67");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=965.70,7691.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1646.81,7668.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5646.85,7561.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=578.85,7347.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1259.92,7275.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2019.57,7364.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2513.90,7412.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3103.98,7498.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3594.78,7446.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4162.69,7429.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4572.72,7457.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5142.84,7467.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1745.00,7071.11");
    }

}

