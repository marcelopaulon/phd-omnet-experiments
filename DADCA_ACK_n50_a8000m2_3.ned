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

network DADCA_ACK_n50_a8000m2_3
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
        @display("i=misc/sensor;p=73.26,7813.52");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=764.37,7888.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1343.60,7805.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4123.30,7974.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4492.45,7787.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4801.24,7967.01");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5194.43,7901.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1898.33,7732.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2618.67,7626.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3157.88,7624.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3739.40,7716.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5784.88,7512.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=544.23,7409.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1143.00,7411.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1552.55,7453.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2118.24,7455.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2444.30,7317.11");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3015.48,7301.52");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4250.07,7435.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4756.28,7436.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1393.45,7079.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2023.81,7102.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3634.94,7198.97");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5235.12,7249.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5617.82,7066.26");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1053.91,6896.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2855.18,6902.35");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4181.76,6899.59");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4587.29,6978.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=587.53,6738.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1140.10,6556.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1563.48,6711.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3206.43,6639.23");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3643.35,6705.85");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4920.51,6744.80");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5518.79,6590.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=226.12,6284.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1832.40,6456.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2338.17,6410.39");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3604.23,6344.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4000.60,6270.95");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4646.12,6456.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=623.96,6101.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=979.22,6206.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1560.71,6232.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3038.27,6182.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4334.58,6084.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5100.40,6243.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5682.39,6080.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1119.12,5845.77");
    }

}

