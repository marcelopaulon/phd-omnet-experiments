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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=107.20,199.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=43.47,191.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=201.78,133.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=176.21,62.44");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=249.63,95.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=317.80,67.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=378.78,102.93");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=446.94,148.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=357.85,181.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=363.78,266.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=375.63,357.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=292.78,229.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=299.98,340.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=233.49,367.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=271.09,439.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=351.71,462.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=411.89,478.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=491.88,436.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=552.82,477.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=627.46,414.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=539.55,554.19");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=473.92,565.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=475.74,498.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=421.69,600.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=489.29,630.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=530.05,699.81");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=441.46,672.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=489.50,350.79");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=313.48,543.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=227.97,570.29");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=166.93,633.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=82.25,657.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=86.46,567.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=91.21,479.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=11.22,573.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=96.99,394.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=166.37,566.36");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=283.51,660.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=334.84,714.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=396.38,727.31");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=376.52,802.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=297.00,794.02");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=483.01,748.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=610.04,643.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=672.30,583.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=627.19,711.22");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=552.14,625.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=646.27,777.92");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=679.08,834.25");
    }

}

