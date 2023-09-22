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

network DADCA_ACK_n50_a8000m2_2
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
        @display("i=misc/sensor;p=148.98,7770.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1293.10,7859.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1963.21,7874.51");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2525.57,7948.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3506.84,7933.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5253.77,7852.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5739.33,7912.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=755.63,7648.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1600.87,7622.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2272.38,7578.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3161.19,7598.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3570.77,7502.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3983.85,7710.65");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4581.06,7635.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=468.36,7302.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3948.58,7276.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4329.50,7369.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4984.98,7476.18");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5510.37,7456.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=117.72,7143.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1111.93,7167.58");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1629.85,7169.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2390.86,7086.92");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3041.95,7137.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4721.60,7149.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5363.08,7091.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5991.26,7129.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=690.69,6807.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2066.80,6793.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3778.40,6828.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5660.17,6868.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=158.90,6673.94");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1028.80,6511.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1492.73,6699.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2442.06,6580.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2834.93,6529.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4424.58,6532.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5144.81,6659.35");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=581.81,6403.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1963.86,6305.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3340.99,6419.10");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3705.78,6222.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5360.50,6331.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5803.51,6213.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=339.60,5987.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1062.69,6003.06");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1589.99,6137.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4985.37,6123.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5337.89,5943.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=779.48,5680.79");
    }

}

